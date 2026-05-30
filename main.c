/*
 * Smart Basil - Automated Plant Watering System with WiFi Web Interface
 * 
 * This ESP32-based system monitors soil moisture, temperature, and humidity to
 * automatically control a water pump. Features include:
 * - I2C LCD display for real-time monitoring
 * - WiFi Access Point mode for remote monitoring
 * - Web server with live data dashboard
 * - Hysteresis-based pump control with environmental adaptation
 * - Safety checks: water level detection, pump interrupt during web access
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_mac.h"

static const char *TAG = "SMART_BASIL";

// ============================================================================
// WiFi AP CONFIGURATION
// ============================================================================
#define AP_SSID         "SmartBasil"           // WiFi network name
#define AP_PASS         "smartbasil123"        // WiFi password (min 8 chars)
#define AP_CHANNEL      1                       // WiFi channel
#define AP_MAX_CLIENTS  4                       // Max connected clients

// ============================================================================
// GLOBAL SENSOR DATA (shared between main loop and HTTP handlers)
// ============================================================================
static struct {
    int water_ok;              // 1=water OK, 0=low
    int soil_percent;          // Soil moisture percentage
    int soil_raw;              // Raw ADC value
    float temp_c;              // Temperature in Celsius
    float temp_f;              // Temperature in Fahrenheit
    float hum_rh;              // Humidity in %RH
    int pump_on;               // 1=pump ON, 0=pump OFF
    int on_threshold;          // Current pump ON threshold
    int off_threshold;         // Current pump OFF threshold
} sensor_data = {0};

// Mutex for thread-safe access to sensor data
static SemaphoreHandle_t sensor_data_mutex = NULL;

// ADC handle for soil moisture sensor
static adc_oneshot_unit_handle_t soil_adc_handle = NULL;

// ============================================================================
// PIN & I2C DEFINITIONS
// ============================================================================

// GPIO pin for water level float switch (active-low input with pull-up)
#define FLOAT_SWITCH_PIN     27

// Soil moisture ADC configuration
#define SOIL_ADC_UNIT        ADC_UNIT_1
#define SOIL_ADC_CHANNEL     ADC_CHANNEL_6    // GPIO34
#define SOIL_ADC_ATTEN       ADC_ATTEN_DB_12  // Range: 0-3.3V

// Soil moisture calibration: raw ADC values at dry and wet conditions
// CALIBRATION NEEDED: Measure raw ADC values for your specific sensor
#define SOIL_ADC_DRY_RAW     2850   // Raw ADC when soil is dry
#define SOIL_ADC_WET_RAW     1375   // Raw ADC when soil is wet (estimate)

// Base pump control thresholds (% soil moisture)
#define SOIL_PERCENT_PUMP_ON_THRESHOLD   40  // Turn pump ON below this
#define SOIL_PERCENT_PUMP_OFF_THRESHOLD  60  // Turn pump OFF above this

// Environmental adjustment bounds: thresholds adjusted within these limits
#define SOIL_PERCENT_PUMP_ON_MIN        30
#define SOIL_PERCENT_PUMP_ON_MAX        50
#define SOIL_PERCENT_PUMP_OFF_MIN       50
#define SOIL_PERCENT_PUMP_OFF_MAX       70

// Water pump control
#define PUMP_GPIO            25      // GPIO for pump relay (IRF520 SIG pin)

// I2C communication configuration (shared by LCD and temperature sensor)
#define I2C_MASTER_SDA       21      // SDA pin
#define I2C_MASTER_SCL       22      // SCL pin
#define I2C_MASTER_PORT      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

// SHT31-D Temperature/Humidity sensor I2C address
#define SHT31_ADDR          0x44

// LCD display (Sunfounder I2C LCD2004) I2C address and control bits
#define LCD_I2C_ADDR        0x27
#define LCD_BACKLIGHT       0x08    // Backlight control bit
#define LCD_ENABLE          0x04    // Enable strobe bit
#define LCD_READ_WRITE      0x02    // Read/Write mode bit
#define LCD_REGISTER_SELECT 0x01    // Instruction/Data mode bit

// Global I2C port handle
static const i2c_port_t i2c_port = I2C_MASTER_PORT;

// Initialize I2C master interface for LCD and SHT31 sensor communication.
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Clamp an integer value between minimum and maximum bounds.
 * @param value The value to clamp
 * @param lo Minimum allowed value
 * @param hi Maximum allowed value
 * @return Clamped value within [lo, hi]
 */
static inline int clamp_i(int value, int lo, int hi)
{
    return value < lo ? lo : value > hi ? hi : value;
}

/**
 * Initialize pump GPIO as output and set initial state to OFF.
 */
static void pump_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PUMP_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PUMP_GPIO, 0);  // Ensure pump is OFF at startup
}

// Set pump state (ON or OFF).
static void pump_set(int on)
{
    gpio_set_level(PUMP_GPIO, on ? 1 : 0);
}

/*
 * Adjust pump control thresholds based on ambient temperature and humidity.
 * - High temp (>=30°C): increase thresholds (water more often)
 * - Low temp (<=18°C): decrease thresholds (water less often)
 * - Low humidity (<=35%): increase thresholds (water more often)
 * - High humidity (>=75%): decrease thresholds (water less often)
 */
static void adjust_soil_thresholds(float temp_c, float hum_rh, int *on_threshold, int *off_threshold)
{
    // Calculate temperature adjustment
    int temp_adj = 0;
    if (!isnan(temp_c)) {
        if (temp_c >= 30.0f) {
            temp_adj = 5;  // Hot: water more
        } else if (temp_c <= 18.0f) {
            temp_adj = -5; // Cold: water less
        }
    }

    // Calculate humidity adjustment
    int hum_adj = 0;
    if (!isnan(hum_rh)) {
        if (hum_rh <= 35.0f) {
            hum_adj = 3;   // Dry: water more
        } else if (hum_rh >= 75.0f) {
            hum_adj = -3;  // Humid: water less
        }
    }

    // Apply adjustments and clamp to safe ranges
    int on_adj = SOIL_PERCENT_PUMP_ON_THRESHOLD + temp_adj + hum_adj;
    int off_adj = SOIL_PERCENT_PUMP_OFF_THRESHOLD + temp_adj + hum_adj;

    *on_threshold = clamp_i(on_adj, SOIL_PERCENT_PUMP_ON_MIN, SOIL_PERCENT_PUMP_ON_MAX);
    *off_threshold = clamp_i(off_adj, SOIL_PERCENT_PUMP_OFF_MIN, SOIL_PERCENT_PUMP_OFF_MAX);
}

/**
 * Convert raw ADC reading to soil moisture percentage using linear interpolation.
 * Uses calibration values: dry=0%, wet=100%
 */
static int soil_raw_to_percent(int raw)
{
    // Invalid reading
    if (raw < 0) {
        return -1;
    }
    // Saturated wet (at or below wet calibration)
    if (raw <= SOIL_ADC_WET_RAW) {
        return 100;
    }
    // Saturated dry (at or above dry calibration)
    if (raw >= SOIL_ADC_DRY_RAW) {
        return 0;
    }
    // Linear interpolation between wet and dry calibration points
    int percent = 100 - ((raw - SOIL_ADC_WET_RAW) * 100) / (SOIL_ADC_DRY_RAW - SOIL_ADC_WET_RAW);
    return clamp_i(percent, 0, 100);
}

// ============================================================================
// LCD DRIVER - Sunfounder I2C LCD2004 (4-bit parallel mode over I2C)
// ============================================================================

// Send raw I2C byte to LCD
static esp_err_t lcd_i2c_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, data, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    }
    i2c_cmd_link_delete(cmd);
    return err;
}

// Pulse LCD enable line to latch data
static esp_err_t lcd_strobe(uint8_t data)
{
    esp_err_t err = lcd_i2c_write(data | LCD_ENABLE | LCD_BACKLIGHT);
    if (err != ESP_OK) return err;
    
    esp_rom_delay_us(1);
    
    err = lcd_i2c_write((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
    if (err != ESP_OK) return err;
    
    esp_rom_delay_us(50);
    return ESP_OK;
}

// Send 4-bit nibble to LCD
static esp_err_t lcd_write4bits(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | mode | LCD_BACKLIGHT;
    return lcd_strobe(data);
}

// Send 8-bit value as two 4-bit nibbles (8-bit mode)
static esp_err_t lcd_send(uint8_t value, uint8_t mode)
{
    esp_err_t err = lcd_write4bits(value & 0xF0, mode);
    if (err != ESP_OK) return err;
    return lcd_write4bits((value << 4) & 0xF0, mode);
}

// Send command to LCD
static esp_err_t lcd_command(uint8_t cmd)
{
    return lcd_send(cmd, 0);
}

// Write single character to LCD
static esp_err_t lcd_write_char(char c)
{
    return lcd_send((uint8_t)c, LCD_REGISTER_SELECT);
}

// Set LCD cursor position (col 0-19, row 0-3)
static esp_err_t lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > 3) row = 3;
    return lcd_command(0x80 | (col + row_offsets[row]));
}

// Print string to LCD
static esp_err_t lcd_print(const char *str)
{
    while (*str) {
        esp_err_t err = lcd_write_char(*str++);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// Clear LCD display
static esp_err_t lcd_clear(void)
{
    esp_err_t err = lcd_command(0x01);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return err;
}

// Turn LCD display off
static esp_err_t lcd_display_off(void)
{
    return lcd_command(0x08);
}

// Turn LCD display on
static esp_err_t lcd_display_on(void)
{
    return lcd_command(0x0C);
}

// Initialize LCD display - sets up 4-bit mode and configures display
static esp_err_t lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));  // Power-on delay

    // Step 1-3: Force 8-bit mode (0x30 = 0011 0000)
    // Repeat 3 times to ensure LCD recognizes the command
    esp_err_t err = lcd_write4bits(0x30, 0);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));

    err = lcd_write4bits(0x30, 0);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(150);

    err = lcd_write4bits(0x30, 0);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(150);

    // Step 4: Set 4-bit mode (0x20 = 0010 0000)
    err = lcd_write4bits(0x20, 0);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(150);

    // Step 5: Configure display settings
    // 0x28 = 4-bit mode, 2 lines, 5x8 font
    err = lcd_command(0x28);
    if (err != ESP_OK) return err;
    
    // 0x08 = Display off
    err = lcd_command(0x08);
    if (err != ESP_OK) return err;
    
    // Clear display
    err = lcd_clear();
    if (err != ESP_OK) return err;
    
    // 0x06 = Auto-increment cursor, no shift
    err = lcd_command(0x06);
    if (err != ESP_OK) return err;
    
    // 0x0C = Display on, no cursor, no blink
    return lcd_command(0x0C);
}

// Display sensor readings on LCD (water level, soil %, temp, humidity)
static esp_err_t lcd_show_status(int water_ok, int soil_percent, float temp_f, float hum_rh)
{
    char buf[21];           // 20 chars + null terminator
    size_t len;
    esp_err_t err;

    // Line 0: Water Level
    err = lcd_set_cursor(0, 0);
    if (err != ESP_OK) return err;
    
    len = snprintf(buf, sizeof(buf), "Water Level:%s", water_ok ? "GOOD" : "LOW");
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memset(buf + len, ' ', 20 - len);  // Pad with spaces
    buf[20] = '\0';
    
    err = lcd_print(buf);
    if (err != ESP_OK) return err;

    // Line 1: Soil Moisture
    err = lcd_set_cursor(0, 1);
    if (err != ESP_OK) return err;
    
    len = snprintf(buf, sizeof(buf), "Soil Moisture:%3d%%", soil_percent >= 0 ? soil_percent : 0);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memset(buf + len, ' ', 20 - len);
    buf[20] = '\0';
    
    err = lcd_print(buf);
    if (err != ESP_OK) return err;

    // Line 2: Temperature
    err = lcd_set_cursor(0, 2);
    if (err != ESP_OK) return err;
    
    if (!isnan(temp_f)) {
        len = snprintf(buf, sizeof(buf), "Temperature:%5.1fF", temp_f);
    } else {
        len = snprintf(buf, sizeof(buf), "Temperature:  --.-F");
    }
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memset(buf + len, ' ', 20 - len);
    buf[20] = '\0';
    
    err = lcd_print(buf);
    if (err != ESP_OK) return err;

    // Line 3: Humidity
    err = lcd_set_cursor(0, 3);
    if (err != ESP_OK) return err;
    
    if (!isnan(hum_rh)) {
        len = snprintf(buf, sizeof(buf), "Humidity:%5.1f%%", hum_rh);
    } else {
        len = snprintf(buf, sizeof(buf), "Humidity:   --.-%%");
    }
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memset(buf + len, ' ', 20 - len);
    buf[20] = '\0';
    
    return lcd_print(buf);
}

// ============================================================================
// TEMPERATURE/HUMIDITY SENSOR - Adafruit SHT31-D I2C Driver
// ============================================================================

// CRC-8 checksum for SHT31 data validation
static uint8_t sht31_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// Convert Celsius to Fahrenheit
static float celsius_to_fahrenheit(float c)
{
    return c * 9.0f / 5.0f + 32.0f;
}

// Read temperature and humidity from SHT31 via I2C (includes CRC verification)
static esp_err_t sht31_read_values(float *temp_c, float *hum_rh)
{
    // SHT31 measurement command (clock stretching enabled)
    const uint8_t cmd[2] = {0x2C, 0x06};
    
    // Send measurement command
    i2c_cmd_handle_t cmdh = i2c_cmd_link_create();
    if (cmdh == NULL) return ESP_ERR_NO_MEM;

    esp_err_t err = i2c_master_start(cmdh);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmdh, (SHT31_ADDR << 1) | I2C_MASTER_WRITE, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_write(cmdh, cmd, sizeof(cmd), true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmdh);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(i2c_port, cmdh, pdMS_TO_TICKS(1000));
    }
    i2c_cmd_link_delete(cmdh);
    if (err != ESP_OK) return err;

    // Wait for measurement to complete (~50ms for normal mode)
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read measurement results: 6 bytes [TEMP_MSB, TEMP_LSB, TEMP_CRC, HUM_MSB, HUM_LSB, HUM_CRC]
    uint8_t data[6];
    cmdh = i2c_cmd_link_create();
    if (cmdh == NULL) return ESP_ERR_NO_MEM;

    err = i2c_master_start(cmdh);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmdh, (SHT31_ADDR << 1) | I2C_MASTER_READ, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_read(cmdh, data, sizeof(data), I2C_MASTER_LAST_NACK);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmdh);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(i2c_port, cmdh, pdMS_TO_TICKS(1000));
    }
    i2c_cmd_link_delete(cmdh);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT31 read I2C error: %s", esp_err_to_name(err));
        return err;
    }

    // Verify checksums
    if (sht31_crc8(data, 2) != data[2] || sht31_crc8(data + 3, 2) != data[5]) {
        ESP_LOGW(TAG, "SHT31 CRC fail data=%02X %02X %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3], data[4], data[5]);
        return ESP_ERR_INVALID_CRC;
    }

    // Parse temperature: 16-bit value, convert from 0-65535 range to -45 to 130°C
    uint16_t temp_raw = (uint16_t)((data[0] << 8) | data[1]);
    *temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);

    // Parse humidity: 16-bit value, convert from 0-65535 range to 0-100% RH
    uint16_t hum_raw = (uint16_t)((data[3] << 8) | data[4]);
    *hum_rh = 100.0f * ((float)hum_raw / 65535.0f);
    
    // Clamp humidity to valid range
    if (*hum_rh > 100.0f) *hum_rh = 100.0f;
    if (*hum_rh < 0.0f) *hum_rh = 0.0f;
    
    return ESP_OK;
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

/*
 * Initialize float switch (water level sensor) as digital input with pull-up.
 */
static void float_switch_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << FLOAT_SWITCH_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Pull high when floating
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * Initialize ADC for soil moisture sensor on ADC_UNIT_1, CHANNEL_6 (GPIO34).
 */
static void soil_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &soil_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = SOIL_ADC_ATTEN,  // 12dB attenuation for 0-3.3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(soil_adc_handle, SOIL_ADC_CHANNEL, &chan_cfg));
}

// ============================================================================
// WiFi EVENT HANDLER
// ============================================================================

// WiFi event handler for AP mode
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" joined, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" left, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

// ============================================================================
// HTTP REQUEST HANDLERS
// ============================================================================

/**
 * HTTP GET handler for /api/status - returns JSON with all sensor data
 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    // Acquire mutex to safely read sensor data
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    
    // Create JSON response manually (no external dependency)
    char json_response[512];
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"water_ok\":%d,"
        "\"soil_percent\":%d,"
        "\"soil_raw\":%d,"
        "\"temp_c\":%.2f,"
        "\"temp_f\":%.2f,"
        "\"hum_rh\":%.2f,"
        "\"pump_on\":%d,"
        "\"pump_on_threshold\":%d,"
        "\"pump_off_threshold\":%d"
        "}",
        sensor_data.water_ok,
        sensor_data.soil_percent,
        sensor_data.soil_raw,
        sensor_data.temp_c,
        sensor_data.temp_f,
        sensor_data.hum_rh,
        sensor_data.pump_on,
        sensor_data.on_threshold,
        sensor_data.off_threshold
    );
    
    // Release mutex
    xSemaphoreGive(sensor_data_mutex);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    return ESP_OK;
}

/**
 * HTTP GET handler for / - returns HTML web interface
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html_page =
        "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Smart Basil - Plant Monitor</title><link href='https://fonts.googleapis.com/css2?family=DM+Mono:wght@400;500&family=Fraunces:ital,wght@0,300;0,600;1,300&display=swap' rel='stylesheet'><style>:root{--green-deep:#1a3a2a;--green-mid:#2d6a4f;--green-light:#52b788;--cream:#f4f0e8;--amber:#e08c3a;--red:#c0392b;--text:#1a3a2a;--text-soft:#4a7c5e;--card-bg:#fff;--border:#d8e8de}*{margin:0;padding:0;box-sizing:border-box}body{font-family:'DM Mono',monospace;background-color:var(--cream);min-height:100vh;color:var(--text);padding:28px 12px}.page-wrap{max-width:680px;margin:0 auto}header{display:flex;align-items:flex-end;gap:12px;margin-bottom:28px}.leaf-icon{width:44px;height:44px;background:var(--green-mid);border-radius:60% 5% 60% 5%;display:flex;align-items:center;justify-content:center;font-size:20px;box-shadow:2px 4px 10px rgba(45,106,79,0.25)}h1{font-family:'Fraunces',serif;font-weight:600;font-size:1.8rem;color:var(--green-deep)}h1 span{display:block;font-style:italic;font-weight:300;font-size:.55em;color:var(--text-soft);margin-top:4px}.status-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:16px}.card{background:var(--card-bg);border:1px solid var(--border);border-radius:12px;padding:14px 12px;position:relative;overflow:hidden}.card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--green-light);border-radius:12px 12px 0 0}.card.alert::before{background:var(--red)}.card.active::before{background:var(--green-mid)}.card-label{font-size:.68em;letter-spacing:.12em;text-transform:uppercase;color:var(--text-soft);margin-bottom:8px}.card-value{font-family:'Fraunces',serif;font-size:1.6rem;font-weight:600;color:var(--green-deep)}.detail-card{background:var(--card-bg);border:1px solid var(--border);border-radius:12px;padding:16px;margin-bottom:16px}.row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid var(--border);font-size:.85em}.row:last-child{border-bottom:none}.row-label{color:var(--text-soft)}.row-val{color:var(--text);font-weight:500}.bar-wrap{display:flex;align-items:center;gap:8px}.bar{width:100px;height:6px;background:var(--border);border-radius:99px;overflow:hidden}.bar-fill{height:100%;background:linear-gradient(90deg,var(--green-mid),var(--green-light));border-radius:99px;transition:width .5s}.timestamp{text-align:center;font-size:.72em;color:var(--text-soft);margin-top:8px}.timestamp.error{color:var(--red)}</style></head><body><div class='page-wrap'><header><div class='leaf-icon'>🌿</div><h1>Smart Basil<span>plant monitoring system</span></h1></header><div class='status-grid'><div class='card' id='card-water'><div class='card-label'>Water Level</div><div class='card-value' id='val-water'>—</div></div><div class='card' id='card-pump'><div class='card-label'>Pump</div><div class='card-value' id='val-pump'>—</div></div><div class='card'><div class='card-label'>Soil Moisture</div><div class='card-value' id='val-soil'>—</div></div><div class='card'><div class='card-label'>Temperature</div><div class='card-value' id='val-temp'>—</div></div></div><div class='detail-card'><h2>Sensor Details</h2><div class='row'><span class='row-label'>Soil moisture</span><div class='bar-wrap'><div class='bar'><div class='bar-fill' id='soil-bar' style='width:0%'></div></div><span class='row-val' id='detail-soil'>—</span></div></div><div class='row'><span class='row-label'>Humidity</span><div class='bar-wrap'><div class='bar'><div class='bar-fill' id='hum-bar' style='width:0%'></div></div><span class='row-val' id='detail-hum'>—</span></div></div><div class='row'><span class='row-label'>Temperature</span><span class='row-val' id='detail-temp'>—</span></div><div class='row'><span class='row-label'>Pump ON threshold</span><span class='row-val' id='detail-on-thresh'>—</span></div><div class='row'><span class='row-label'>Pump OFF threshold</span><span class='row-val' id='detail-off-thresh'>—</span></div><div class='row'><span class='row-label'>Raw ADC value</span><span class='row-val' id='detail-raw'>—</span></div></div><div class='timestamp' id='timestamp'>Waiting for data…</div></div><script>function render(d){const ts=document.getElementById('timestamp');if(!d){ts.textContent='No data — connect to ESP32 AP';ts.className='timestamp error';return}const cardW=document.getElementById('card-water'),valW=document.getElementById('val-water');if(d.water_ok){cardW.className='card active';valW.textContent='GOOD'}else{cardW.className='card alert';valW.textContent='LOW'}const cardP=document.getElementById('card-pump'),valP=document.getElementById('val-pump');if(d.pump_on){cardP.className='card active';valP.textContent='ON'}else{cardP.className='card';valP.textContent='OFF'}document.getElementById('val-soil').textContent=(d.soil_percent!==undefined)?d.soil_percent+'%':'—';document.getElementById('val-temp').textContent=(d.temp_f!==undefined)?d.temp_f.toFixed(1)+'°F':'—';document.getElementById('detail-soil').textContent=(d.soil_percent!==undefined)?d.soil_percent+'%':'—';document.getElementById('detail-hum').textContent=(d.hum_rh!==undefined)?d.hum_rh.toFixed(1)+'%':'—';document.getElementById('detail-temp').textContent=(d.temp_f!==undefined&&d.temp_c!==undefined)?d.temp_f.toFixed(1)+'°F / '+d.temp_c.toFixed(1)+'°C':'—';document.getElementById('detail-on-thresh').textContent=(d.pump_on_threshold!==undefined)?d.pump_on_threshold+'%':'—';document.getElementById('detail-off-thresh').textContent=(d.pump_off_threshold!==undefined)?d.pump_off_threshold+'%':'—';document.getElementById('detail-raw').textContent=(d.soil_raw!==undefined)?d.soil_raw:'—';document.getElementById('soil-bar').style.width=((d.soil_percent||0)+'%');document.getElementById('hum-bar').style.width=((d.hum_rh||0)+'%');ts.textContent='Last updated: '+new Date().toLocaleTimeString();ts.className='timestamp'}function fetchData(){fetch('/api/status').then(r=>{if(!r.ok)throw new Error('no data');return r.json()}).then(data=>render(data)).catch(()=>render(null))}fetchData();setInterval(fetchData,2000)</script></body></html>";
        
httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    
    return ESP_OK;
}

// Start HTTP server with dashboard and API endpoints
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 2;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);
        
        httpd_uri_t status_api = {
            .uri       = "/api/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_api);
        
        ESP_LOGI(TAG, "Web server started");
    }
    return server;
}

// ============================================================================
// WiFi INITIALIZATION
// ============================================================================

/**
 * Initialize WiFi in AP (Access Point) mode
 */
static void wifi_init_ap(void)
{
    // Initialize NVS for WiFi storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create WiFi AP interface
    esp_netif_create_default_wifi_ap();
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    // Configure WiFi AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASS,
            .max_connection = AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP initialized");
    ESP_LOGI(TAG, "  SSID: %s", AP_SSID);
    ESP_LOGI(TAG, "  Password: %s", AP_PASS);
    ESP_LOGI(TAG, "  Visit http://192.168.4.1 from a connected device");
}

// ============================================================================
// MAIN APPLICATION - Automated Plant Watering Control Loop
// ============================================================================

void app_main(void)
{
    // Set log verbosity
    esp_log_level_set("*", ESP_LOG_INFO);

    // ========== MUTEX INITIALIZATION ==========
    sensor_data_mutex = xSemaphoreCreateMutex();
    if (sensor_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data mutex");
        return;
    }

    // ========== HARDWARE INITIALIZATION ==========
    ESP_LOGI(TAG, "Initializing hardware...");

    ESP_ERROR_CHECK(i2c_master_init());      // I2C bus for LCD and SHT31
    
    esp_err_t lcd_ret = lcd_init();           // Initialize LCD display
    if (lcd_ret != ESP_OK) {
        ESP_LOGW(TAG, "LCD initialization failed: %s (proceeding without display)", esp_err_to_name(lcd_ret));
    }
    
    float_switch_init();                      // Water level sensor
    soil_adc_init();                          // Soil moisture ADC
    pump_gpio_init();                         // Water pump GPIO

    // ========== WiFi & WEB SERVER INITIALIZATION ==========
    ESP_LOGI(TAG, "Initializing WiFi and web server...");
    wifi_init_ap();
    httpd_handle_t server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
    }

    ESP_LOGI(TAG, "Initialization complete.");
    ESP_LOGI(TAG, "Entering main control loop...");
    
    // ========== PERSISTENT STATE VARIABLES ==========
    int pump_on = 0;                    // Current pump state
    bool lcd_enabled = true;            // Track LCD state

    // ========== MAIN CONTROL LOOP ==========
    while (1) {
        // ===== READ SENSOR INPUTS =====
        // Float switch: 1=water OK, 0=low
        int water_ok = gpio_get_level(FLOAT_SWITCH_PIN);

        // Read raw soil moisture ADC (0-4095)
        int soil_raw = -1;
        esp_err_t soil_ret = adc_oneshot_read(soil_adc_handle, SOIL_ADC_CHANNEL, &soil_raw);
        if (soil_ret != ESP_OK) soil_raw = -1;
        
        // Convert to percentage using calibration
        int soil_percent = soil_raw_to_percent(soil_raw);

        // Read temperature and humidity from SHT31
        float temp_c = NAN;
        float temp_f = NAN;
        float hum_rh = NAN;
        esp_err_t sht_ret = sht31_read_values(&temp_c, &hum_rh);
        if (sht_ret == ESP_OK) {
            temp_f = celsius_to_fahrenheit(temp_c);
        }

        // ===== CALCULATE ADAPTIVE PUMP THRESHOLDS =====
        // Adjust thresholds based on temperature and humidity
        int on_threshold = SOIL_PERCENT_PUMP_ON_THRESHOLD;
        int off_threshold = SOIL_PERCENT_PUMP_OFF_THRESHOLD;
        adjust_soil_thresholds(temp_c, hum_rh, &on_threshold, &off_threshold);

        // ===== UPDATE SHARED SENSOR DATA (for web server) =====
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        sensor_data.water_ok = water_ok;
        sensor_data.soil_percent = soil_percent;
        sensor_data.soil_raw = soil_raw;
        sensor_data.temp_c = temp_c;
        sensor_data.temp_f = temp_f;
        sensor_data.hum_rh = hum_rh;
        sensor_data.pump_on = pump_on;
        sensor_data.on_threshold = on_threshold;
        sensor_data.off_threshold = off_threshold;
        xSemaphoreGive(sensor_data_mutex);

        // Log sensor readings
        ESP_LOGI(TAG, "Float=%d Soil=%d%% (raw=%d) Temp=%.2fF Hum=%.2f%% (SHT31=%s)",
                 water_ok,
                 soil_percent,
                 soil_raw,
                 temp_f,
                 hum_rh,
                 sht_ret == ESP_OK ? "OK" : "ERR");

        // ===== PUMP CONTROL LOGIC =====
        // Hysteresis-based control with water level safety check
        bool pump_state_changed = false;
        
        // TURN PUMP ON: soil below threshold AND water available AND pump is off
        if (water_ok && soil_percent < on_threshold && !pump_on) {
            // Turn off LCD during pump operation to prevent electrical interference
            if (lcd_display_off() == ESP_OK) {
                lcd_enabled = false;
                ESP_LOGI(TAG, "LCD disabled (pump running)");
            }

            vTaskDelay(pdMS_TO_TICKS(500));

            pump_set(1);
            pump_on = 1;
            pump_state_changed = true;
            ESP_LOGI(TAG, "Pump ON (soil moisture %d%% < threshold %d%%)", soil_percent, on_threshold);
            
            // Let power supply stabilize
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // TURN PUMP OFF: soil above threshold OR no water available
        if ((soil_percent > off_threshold && pump_on) || !water_ok) {
            pump_set(0);
            pump_on = 0;
            pump_state_changed = true;
            ESP_LOGI(TAG, "Pump OFF (soil moisture %d%% > threshold %d%% OR water low)", soil_percent, off_threshold);
            
            // Extended delay for electrical interference to dissipate
            vTaskDelay(pdMS_TO_TICKS(500));

            // Reset and reactivate LCD with retry logic (pump interference may cause errors)
            bool lcd_reset_success = false;
            for (int retry = 0; retry < 3 && !lcd_reset_success; retry++) {
                if (lcd_clear() == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    if (lcd_display_on() == ESP_OK) {
                        lcd_enabled = true;
                        lcd_reset_success = true;
                        ESP_LOGI(TAG, "LCD reactivated successfully (attempt %d)", retry + 1);
                    }
                }
                if (!lcd_reset_success && retry < 2) {
                    vTaskDelay(pdMS_TO_TICKS(200));  // Wait before retry
                }
            }
            if (!lcd_reset_success) {
                ESP_LOGW(TAG, "LCD reactivation failed after 3 attempts");
            }
        }

        // ===== DISPLAY UPDATE =====
        // Update LCD with current status (only when enabled)
        if (lcd_enabled) {
            int retry_count = 0;
            // More retries immediately after pump state change (interference may linger)
            const int max_retries = pump_state_changed ? 5 : 2;

            // Attempt to update display with retries
            while (retry_count < max_retries) {
                if (lcd_show_status(water_ok, soil_percent, temp_f, hum_rh) == ESP_OK) {
                    break;  // Success
                }
                ESP_LOGW(TAG, "LCD update failed (attempt %d/%d)", retry_count + 1, max_retries);
                retry_count++;
                if (retry_count < max_retries) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            // If all retries fail, attempt full LCD reinitialization
            if (retry_count >= max_retries) {
                ESP_LOGE(TAG, "LCD update failed after %d attempts - reinitializing", max_retries);
                if (lcd_init() == ESP_OK) {
                    ESP_LOGI(TAG, "LCD reinitialized");
                    // Try one more status update after reinit
                    if (lcd_show_status(water_ok, soil_percent, temp_f, hum_rh) == ESP_OK) {
                        ESP_LOGI(TAG, "LCD update successful after reinitialization");
                    }
                } else {
                    ESP_LOGE(TAG, "LCD reinitialization failed");
                }
            }
        }

        // ===== LOOP CONTROL =====
        // Main loop period: 2 seconds between sensor reads
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
