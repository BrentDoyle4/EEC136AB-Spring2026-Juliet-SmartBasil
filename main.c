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
#define SOIL_PERCENT_PUMP_ON_THRESHOLD   10  // Turn pump ON below this
#define SOIL_PERCENT_PUMP_OFF_THRESHOLD  20  // Turn pump OFF above this

// Environmental adjustment bounds: thresholds adjusted within these limits
#define SOIL_PERCENT_PUMP_ON_MIN        10
#define SOIL_PERCENT_PUMP_ON_MAX        60
#define SOIL_PERCENT_PUMP_OFF_MIN       20
#define SOIL_PERCENT_PUMP_OFF_MAX       75

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

/**
 * Initialize I2C master interface for LCD and SHT31 sensor communication.
 * @return ESP_OK on success, error code otherwise
 */
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

/**
 * Set pump state (ON or OFF).
 * @param on 1 to turn pump ON, 0 to turn pump OFF
 */
static void pump_set(int on)
{
    gpio_set_level(PUMP_GPIO, on ? 1 : 0);
}

/**
 * Adjust pump control thresholds based on ambient temperature and humidity.
 * - High temp (>=30°C): increase thresholds (water more often)
 * - Low temp (<=18°C): decrease thresholds (water less often)
 * - Low humidity (<=35%): increase thresholds (water more often)
 * - High humidity (>=75%): decrease thresholds (water less often)
 * @param temp_c Ambient temperature in Celsius (NaN if unavailable)
 * @param hum_rh Ambient humidity in % RH (NaN if unavailable)
 * @param on_threshold Output: adjusted pump ON threshold (%)
 * @param off_threshold Output: adjusted pump OFF threshold (%)
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
 * @param raw Raw ADC value from soil moisture sensor
 * @return Soil moisture as percentage (0-100), or -1 if invalid
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

/**
 * Write raw data byte to LCD via I2C.
 * @param data 8-bit value to send (includes command/data + control bits)
 * @return ESP_OK on success, error code otherwise
 */
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

/**
 * Pulse LCD enable line to latch data. Handles 4-bit parallel protocol.
 * @param data Value with high nibble set
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_strobe(uint8_t data)
{
    // Set enable high, latch data
    esp_err_t err = lcd_i2c_write(data | LCD_ENABLE | LCD_BACKLIGHT);
    if (err != ESP_OK) return err;
    
    esp_rom_delay_us(1);  // Hold time
    
    // Set enable low, complete pulse
    err = lcd_i2c_write((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
    if (err != ESP_OK) return err;
    
    esp_rom_delay_us(50);  // Setup time for next operation
    return ESP_OK;
}

/**
 * Send 4-bit nibble to LCD with optional mode.
 * @param nibble 4-bit value (upper 4 bits must be set)
 * @param mode Mode bits: 0 for command, LCD_REGISTER_SELECT for data
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_write4bits(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | mode | LCD_BACKLIGHT;
    return lcd_strobe(data);
}

/**
 * Send 8-bit value to LCD as two 4-bit nibbles (8-bit mode).
 * @param value 8-bit command or data byte
 * @param mode Mode bits: 0 for command, LCD_REGISTER_SELECT for data
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_send(uint8_t value, uint8_t mode)
{
    // Send high nibble
    esp_err_t err = lcd_write4bits(value & 0xF0, mode);
    if (err != ESP_OK) return err;
    
    // Send low nibble (shifted to upper 4 bits)
    return lcd_write4bits((value << 4) & 0xF0, mode);
}

/**
 * Send instruction/command to LCD (data mode = 0).
 * @param cmd Command byte
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_command(uint8_t cmd)
{
    return lcd_send(cmd, 0);
}

/**
 * Write single character to LCD at current cursor position.
 * @param c Character to display
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_write_char(char c)
{
    return lcd_send((uint8_t)c, LCD_REGISTER_SELECT);
}

/**
 * Set LCD cursor position.
 * @param col Column (0-19)
 * @param row Row (0-3)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_set_cursor(uint8_t col, uint8_t row)
{
    // DDRAM addresses for each row (4-line display)
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    
    // Bounds check
    if (row > 3) row = 3;
    
    // Set DDRAM address command: 0x80 | address
    return lcd_command(0x80 | (col + row_offsets[row]));
}

/**
 * Write null-terminated string to LCD at current cursor position.
 * @param str String to display
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_print(const char *str)
{
    while (*str) {
        esp_err_t err = lcd_write_char(*str++);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

/**
 * Clear LCD display and move cursor to home position.
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_clear(void)
{
    esp_err_t err = lcd_command(0x01);  // Clear display command
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));   // Clear operation takes ~1.5ms
    }
    return err;
}

/**
 * Turn LCD display OFF (data preserved).
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_display_off(void)
{
    return lcd_command(0x08);  // Display off, cursor off, blink off
}

/**
 * Turn LCD display ON.
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t lcd_display_on(void)
{
    return lcd_command(0x0C);  // Display on, cursor off, blink off
}

/**
 * Initialize LCD display: set 4-bit mode, configure settings, clear screen.
 * Follows HD44780 initialization sequence for 4-bit mode.
 * @return ESP_OK on success, error code otherwise
 */
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

/**
 * Display system status on LCD (4 lines, 20 chars per line).
 * Line 0: Water level status
 * Line 1: Soil moisture percentage
 * Line 2: Temperature in Fahrenheit
 * Line 3: Relative humidity
 * @param water_ok 1=water OK, 0=low
 * @param soil_percent Soil moisture (0-100%)
 * @param temp_f Temperature in Fahrenheit (NaN if unavailable)
 * @param hum_rh Humidity in % RH (NaN if unavailable)
 * @return ESP_OK on success, error code otherwise
 */
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

/**
 * Compute CRC-8 checksum for SHT31 data validation.
 * Polynomial: 0x31 (CRC-8/MAXIM)
 * @param data Byte array
 * @param len Number of bytes
 * @return CRC-8 checksum
 */
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

/**
 * Convert temperature from Celsius to Fahrenheit.
 * Formula: F = C * (9/5) + 32
 * @param c Temperature in Celsius
 * @return Temperature in Fahrenheit
 */
static float celsius_to_fahrenheit(float c)
{
    return c * 9.0f / 5.0f + 32.0f;
}

/**
 * Read temperature and humidity from SHT31 sensor via I2C.
 * Sends measurement command, waits for conversion, reads 6 bytes with CRC checks.
 * @param temp_c Output: temperature in Celsius
 * @param hum_rh Output: relative humidity in %
 * @return ESP_OK on success, error code otherwise
 */
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

/**
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

/**
 * WiFi event handler for AP mode.
 */
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
        "<!DOCTYPE html>\n"
        "        <html lang=\"en\">\n"
        "        <head>\n"
        "        <meta charset=\"UTF-8\">\n"
        "        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
        "        <title>Smart Basil - Plant Monitor</title>\n"
        "        <link href=\"https://fonts.googleapis.com/css2?family=DM+Mono:wght@400;500&family=Fraunces:ital,wght@0,300;0,600;1,300&display=swap\" rel=\"stylesheet\">\n"
        "        <style>\n"
        "            :root {\n"
        "            --green-deep:  #1a3a2a;\n"
        "            --green-mid:   #2d6a4f;\n"
        "            --green-light: #52b788;\n"
        "            --green-pale:  #b7e4c7;\n"
        "            --cream:       #f4f0e8;\n"
        "            --amber:       #e08c3a;\n"
        "            --red:         #c0392b;\n"
        "            --text:        #1a3a2a;\n"
        "            --text-soft:   #4a7c5e;\n"
        "            --card-bg:     #ffffff;\n"
        "            --border:      #d8e8de;\n"
        "            }\n"
        "\n"
        "            * { margin: 0; padding: 0; box-sizing: border-box; }\n"
        "\n"
        "            body {\n"
        "            font-family: 'DM Mono', monospace;\n"
        "            background-color: var(--cream);\n"
        "            background-image:\n"
        "                radial-gradient(ellipse 80% 60% at 10% 0%, rgba(82,183,136,0.15) 0%, transparent 60%),\n"
        "                radial-gradient(ellipse 60% 80% at 90% 100%, rgba(45,106,79,0.12) 0%, transparent 60%);\n"
        "            min-height: 100vh;\n"
        "            color: var(--text);\n"
        "            padding: 40px 20px 60px;\n"
        "            }\n"
        "\n"
        "            .page-wrap {\n"
        "            max-width: 680px;\n"
        "            margin: 0 auto;\n"
        "            }\n"
        "\n"
        "            /* ── Header ── */\n"
        "            header {\n"
        "            display: flex;\n"
        "            align-items: flex-end;\n"
        "            gap: 14px;\n"
        "            margin-bottom: 44px;\n"
        "            }\n"
        "            .leaf-icon {\n"
        "            width: 52px;\n"
        "            height: 52px;\n"
        "            background: var(--green-mid);\n"
        "            border-radius: 60% 5% 60% 5%;\n"
        "            display: flex;\n"
        "            align-items: center;\n"
        "            justify-content: center;\n"
        "            font-size: 24px;\n"
        "            flex-shrink: 0;\n"
        "            box-shadow: 2px 4px 12px rgba(45,106,79,0.3);\n"
        "            }\n"
        "            h1 {\n"
        "            font-family: 'Fraunces', serif;\n"
        "            font-weight: 600;\n"
        "            font-size: 2.4em;\n"
        "            color: var(--green-deep);\n"
        "            line-height: 1;\n"
        "            }\n"
        "            h1 span {\n"
        "            display: block;\n"
        "            font-style: italic;\n"
        "            font-weight: 300;\n"
        "            font-size: 0.55em;\n"
        "            color: var(--text-soft);\n"
        "            letter-spacing: 0.04em;\n"
        "            margin-top: 4px;\n"
        "            }\n"
        "\n"
        "            /* ── Mock-data notice ── */\n"
        "            .mock-banner {\n"
        "            background: #fffbe6;\n"
        "            border: 1px solid #f0d060;\n"
        "            border-radius: 8px;\n"
        "            padding: 10px 16px;\n"
        "            font-size: 0.75em;\n"
        "            color: #7a5c00;\n"
        "            margin-bottom: 28px;\n"
        "            display: flex;\n"
        "            align-items: center;\n"
        "            gap: 8px;\n"
        "            }\n"
        "\n"
        "            /* ── Status grid ── */\n"
        "            .status-grid {\n"
        "            display: grid;\n"
        "            grid-template-columns: 1fr 1fr;\n"
        "            gap: 14px;\n"
        "            margin-bottom: 20px;\n"
        "            }\n"
        "\n"
        "            .card {\n"
        "            background: var(--card-bg);\n"
        "            border: 1px solid var(--border);\n"
        "            border-radius: 16px;\n"
        "            padding: 20px 18px 16px;\n"
        "            position: relative;\n"
        "            overflow: hidden;\n"
        "            transition: box-shadow 0.2s;\n"
        "            }\n"
        "            .card::before {\n"
        "            content: '';\n"
        "            position: absolute;\n"
        "            top: 0; left: 0; right: 0;\n"
        "            height: 3px;\n"
        "            background: var(--green-light);\n"
        "            border-radius: 16px 16px 0 0;\n"
        "            }\n"
        "            .card.alert::before { background: var(--red); }\n"
        "            .card.warn::before  { background: var(--amber); }\n"
        "            .card.active::before { background: var(--green-mid); }\n"
        "\n"
        "            .card-label {\n"
        "            font-size: 0.68em;\n"
        "            letter-spacing: 0.12em;\n"
        "            text-transform: uppercase;\n"
        "            color: var(--text-soft);\n"
        "            margin-bottom: 10px;\n"
        "            }\n"
        "            .card-value {\n"
        "            font-family: 'Fraunces', serif;\n"
        "            font-size: 2em;\n"
        "            font-weight: 600;\n"
        "            color: var(--green-deep);\n"
        "            line-height: 1;\n"
        "            }\n"
        "            .card-value.alert { color: var(--red); }\n"
        "            .card-value.warn  { color: var(--amber); }\n"
        "\n"
        "            /* ── Detail table ── */\n"
        "            .detail-card {\n"
        "            background: var(--card-bg);\n"
        "            border: 1px solid var(--border);\n"
        "            border-radius: 16px;\n"
        "            padding: 24px 22px;\n"
        "            margin-bottom: 20px;\n"
        "            }\n"
        "            .detail-card h2 {\n"
        "            font-family: 'Fraunces', serif;\n"
        "            font-size: 1em;\n"
        "            font-weight: 600;\n"
        "            color: var(--green-deep);\n"
        "            margin-bottom: 18px;\n"
        "            letter-spacing: 0.01em;\n"
        "            }\n"
        "            .row {\n"
        "            display: flex;\n"
        "            justify-content: space-between;\n"
        "            align-items: center;\n"
        "            padding: 11px 0;\n"
        "            border-bottom: 1px solid var(--border);\n"
        "            font-size: 0.85em;\n"
        "            }\n"
        "            .row:last-child { border-bottom: none; }\n"
        "            .row-label { color: var(--text-soft); }\n"
        "            .row-val { color: var(--text); font-weight: 500; }\n"
        "\n"
        "            /* progress bar */\n"
        "            .bar-wrap { display: flex; align-items: center; gap: 10px; }\n"
        "            .bar {\n"
        "            width: 100px;\n"
        "            height: 6px;\n"
        "            background: var(--border);\n"
        "            border-radius: 99px;\n"
        "            overflow: hidden;\n"
        "            }\n"
        "            .bar-fill {\n"
        "            height: 100%;\n"
        "            background: linear-gradient(90deg, var(--green-mid), var(--green-light));\n"
        "            border-radius: 99px;\n"
        "            transition: width 0.5s ease;\n"
        "            }\n"
        "            .bar-fill.low { background: linear-gradient(90deg, var(--amber), #f4a53a); }\n"
        "\n"
        "            /* ── Manual pump card ── */\n"
        "            .pump-card {\n"
        "            background: var(--card-bg);\n"
        "            border: 1px solid var(--border);\n"
        "            border-radius: 16px;\n"
        "            padding: 24px 22px;\n"
        "            margin-bottom: 20px;\n"
        "            }\n"
        "            .pump-card h2 {\n"
        "            font-family: 'Fraunces', serif;\n"
        "            font-size: 1em;\n"
        "            font-weight: 600;\n"
        "            color: var(--green-deep);\n"
        "            margin-bottom: 6px;\n"
        "            }\n"
        "            .pump-card p {\n"
        "            font-size: 0.78em;\n"
        "            color: var(--text-soft);\n"
        "            margin-bottom: 20px;\n"
        "            line-height: 1.5;\n"
        "            }\n"
        "\n"
        "            .pump-controls {\n"
        "            display: flex;\n"
        "            align-items: center;\n"
        "            gap: 18px;\n"
        "            flex-wrap: wrap;\n"
        "            }\n"
        "\n"
        "            /* toggle switch */\n"
        "            .toggle-wrap {\n"
        "            display: flex;\n"
        "            align-items: center;\n"
        "            gap: 12px;\n"
        "            }\n"
        "            .toggle-label {\n"
        "            font-size: 0.82em;\n"
        "            color: var(--text-soft);\n"
        "            }\n"
        "            .toggle {\n"
        "            position: relative;\n"
        "            width: 56px;\n"
        "            height: 30px;\n"
        "            cursor: pointer;\n"
        "            }\n"
        "            .toggle input { opacity: 0; width: 0; height: 0; }\n"
        "            .slider {\n"
        "            position: absolute;\n"
        "            inset: 0;\n"
        "            background: var(--border);\n"
        "            border-radius: 99px;\n"
        "            transition: background 0.25s;\n"
        "            }\n"
        "            .slider::after {\n"
        "            content: '';\n"
        "            position: absolute;\n"
        "            width: 22px;\n"
        "            height: 22px;\n"
        "            left: 4px;\n"
        "            top: 4px;\n"
        "            background: white;\n"
        "            border-radius: 50%;\n"
        "            transition: transform 0.25s;\n"
        "            box-shadow: 0 1px 4px rgba(0,0,0,0.2);\n"
        "            }\n"
        "            .toggle input:checked + .slider {\n"
        "            background: var(--green-mid);\n"
        "            }\n"
        "            .toggle input:checked + .slider::after {\n"
        "            transform: translateX(26px);\n"
        "            }\n"
        "\n"
        "            /* pump status pill */\n"
        "            .pump-pill {\n"
        "            display: inline-flex;\n"
        "            align-items: center;\n"
        "            gap: 7px;\n"
        "            padding: 6px 14px;\n"
        "            border-radius: 99px;\n"
        "            font-size: 0.78em;\n"
        "            font-weight: 500;\n"
        "            letter-spacing: 0.05em;\n"
        "            transition: all 0.25s;\n"
        "            }\n"
        "            .pump-pill.off {\n"
        "            background: #f0f4f1;\n"
        "            color: var(--text-soft);\n"
        "            }\n"
        "            .pump-pill.on {\n"
        "            background: #e6f4ec;\n"
        "            color: var(--green-mid);\n"
        "            }\n"
        "            .pump-dot {\n"
        "            width: 8px;\n"
        "            height: 8px;\n"
        "            border-radius: 50%;\n"
        "            background: currentColor;\n"
        "            }\n"
        "            .pump-pill.on .pump-dot {\n"
        "            animation: pulse 1.2s infinite;\n"
        "            }\n"
        "            @keyframes pulse {\n"
        "            0%,100% { opacity: 1; transform: scale(1); }\n"
        "            50%      { opacity: 0.5; transform: scale(0.7); }\n"
        "            }\n"
        "\n"
        "            /* manual override indicator */\n"
        "            .override-tag {\n"
        "            font-size: 0.72em;\n"
        "            background: #fff3e0;\n"
        "            color: var(--amber);\n"
        "            padding: 3px 9px;\n"
        "            border-radius: 99px;\n"
        "            border: 1px solid #f0c060;\n"
        "            display: none;\n"
        "            }\n"
        "            .override-tag.visible { display: inline-block; }\n"
        "\n"
        "            /* ── Timestamp ── */\n"
        "            .timestamp {\n"
        "            text-align: center;\n"
        "            font-size: 0.72em;\n"
        "            color: var(--text-soft);\n"
        "            margin-top: 10px;\n"
        "            }\n"
        "            .timestamp.error { color: var(--red); }\n"
        "        </style>\n"
        "        </head>\n"
        "        <body>\n"
        "        <div class=\"page-wrap\">\n"
        "\n"
        "            <header>\n"
        "            <div class=\"leaf-icon\">🌿</div>\n"
        "            <h1>Smart Basil\n"
        "                <span>plant monitoring system</span>\n"
        "            </h1>\n"
        "            </header>\n"
        "\n"
        "            <div class=\"mock-banner\">\n"
        "            ⚠️ <span><strong>Demo mode</strong> — simulated sensor data. Connect to the ESP32 AP to get live readings.</span>\n"
        "            </div>\n"
        "\n"
        "            <!-- Status cards -->\n"
        "            <div class=\"status-grid\">\n"
        "            <div class=\"card\" id=\"card-water\">\n"
        "                <div class=\"card-label\">Water Level</div>\n"
        "                <div class=\"card-value\" id=\"val-water\">—</div>\n"
        "            </div>\n"
        "            <div class=\"card\" id=\"card-pump\">\n"
        "                <div class=\"card-label\">Pump</div>\n"
        "                <div class=\"card-value\" id=\"val-pump\">—</div>\n"
        "            </div>\n"
        "            <div class=\"card\">\n"
        "                <div class=\"card-label\">Soil Moisture</div>\n"
        "                <div class=\"card-value\" id=\"val-soil\">—</div>\n"
        "            </div>\n"
        "            <div class=\"card\">\n"
        "                <div class=\"card-label\">Temperature</div>\n"
        "                <div class=\"card-value\" id=\"val-temp\">—</div>\n"
        "            </div>\n"
        "            </div>\n"
        "\n"
        "            <!-- Manual pump control -->\n"
        "            <div class=\"pump-card\">\n"
        "            <h2>💧 Manual Pump Control</h2>\n"
        "            <p>Override the automatic watering schedule. When manual mode is active, the pump ignores soil moisture thresholds.</p>\n"
        "            <div class=\"pump-controls\">\n"
        "                <label class=\"toggle\">\n"
        "                <input type=\"checkbox\" id=\"pump-toggle\" onchange=\"handlePumpToggle(this.checked)\">\n"
        "                <span class=\"slider\"></span>\n"
        "                </label>\n"
        "                <span class=\"toggle-label\" id=\"toggle-label\">Pump off</span>\n"
        "                <div class=\"pump-pill off\" id=\"pump-pill\">\n"
        "                <div class=\"pump-dot\"></div>\n"
        "                <span id=\"pill-text\">IDLE</span>\n"
        "                </div>\n"
        "                <span class=\"override-tag\" id=\"override-tag\">MANUAL OVERRIDE</span>\n"
        "            </div>\n"
        "            </div>\n"
        "\n"
        "            <!-- Sensor detail rows -->\n"
        "            <div class=\"detail-card\">\n"
        "            <h2>Sensor Details</h2>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Soil moisture</span>\n"
        "                <div class=\"bar-wrap\">\n"
        "                <div class=\"bar\"><div class=\"bar-fill\" id=\"soil-bar\" style=\"width:0%\"></div></div>\n"
        "                <span class=\"row-val\" id=\"detail-soil\">—</span>\n"
        "                </div>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Humidity</span>\n"
        "                <div class=\"bar-wrap\">\n"
        "                <div class=\"bar\"><div class=\"bar-fill\" id=\"hum-bar\" style=\"width:0%\"></div></div>\n"
        "                <span class=\"row-val\" id=\"detail-hum\">—</span>\n"
        "                </div>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Temperature</span>\n"
        "                <span class=\"row-val\" id=\"detail-temp\">—</span>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Pump ON threshold</span>\n"
        "                <span class=\"row-val\" id=\"detail-on-thresh\">—</span>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Pump OFF threshold</span>\n"
        "                <span class=\"row-val\" id=\"detail-off-thresh\">—</span>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"row\">\n"
        "                <span class=\"row-label\">Raw ADC value</span>\n"
        "                <span class=\"row-val\" id=\"detail-raw\">—</span>\n"
        "            </div>\n"
        "            </div>\n"
        "\n"
        "            <div class=\"timestamp\" id=\"timestamp\">Waiting for data…</div>\n"
        "\n"
        "        </div>\n"
        "\n"
        "        <script>\n"
        "            // ── State ──────────────────────────────────────────────────────────────\n"
        "            let manualOverride = false;   // true when user has toggled pump manually\n"
        "            let manualPumpOn   = false;   // desired pump state in manual mode\n"
        "\n"
        "            // ── Mock data (simulates live ESP32 readings) ──────────────────────────\n"
        "            let mockState = {\n"
        "            water_ok:           1,\n"
        "            soil_percent:       14,       // starts low to show pump-on scenario\n"
        "            soil_raw:           2600,\n"
        "            temp_c:             22.4,\n"
        "            temp_f:             72.3,\n"
        "            hum_rh:             55.0,\n"
        "            pump_on:            0,\n"
        "            pump_on_threshold:  10,\n"
        "            pump_off_threshold: 20,\n"
        "            };\n"
        "\n"
        "            // Slowly drift soil moisture to demonstrate automation\n"
        "            function tickMockData() {\n"
        "            if (mockState.pump_on && !manualOverride) {\n"
        "                mockState.soil_percent = Math.min(100, mockState.soil_percent + 2);\n"
        "            } else if (!mockState.pump_on && !manualOverride) {\n"
        "                mockState.soil_percent = Math.max(0, mockState.soil_percent - 1);\n"
        "            }\n"
        "            mockState.hum_rh = Math.min(100, Math.max(20,\n"
        "                mockState.hum_rh + (Math.random() - 0.5) * 0.4));\n"
        "            mockState.temp_c = parseFloat(\n"
        "                (mockState.temp_c + (Math.random() - 0.5) * 0.1).toFixed(1));\n"
        "            mockState.temp_f = mockState.temp_c * 9/5 + 32;\n"
        "            mockState.soil_raw = Math.round(\n"
        "                2850 - (mockState.soil_percent / 100) * (2850 - 1375));\n"
        "            }\n"
        "\n"
        "            function autoControl(d) {\n"
        "            // Mirrors the C hysteresis logic\n"
        "            if (!manualOverride) {\n"
        "                if (d.water_ok && d.soil_percent < d.pump_on_threshold && !d.pump_on) {\n"
        "                d.pump_on = 1;\n"
        "                } else if ((d.soil_percent > d.pump_off_threshold && d.pump_on) || !d.water_ok) {\n"
        "                d.pump_on = 0;\n"
        "                }\n"
        "            } else {\n"
        "                d.pump_on = manualPumpOn ? 1 : 0;\n"
        "            }\n"
        "            }\n"
        "\n"
        "            // ── Render ─────────────────────────────────────────────────────────────\n"
        "            function render(d) {\n"
        "            // Water card\n"
        "            const cardW = document.getElementById('card-water');\n"
        "            const valW  = document.getElementById('val-water');\n"
        "            if (d.water_ok) {\n"
        "                cardW.className = 'card active';\n"
        "                valW.textContent = 'GOOD';\n"
        "                valW.className = 'card-value';\n"
        "            } else {\n"
        "                cardW.className = 'card alert';\n"
        "                valW.textContent = 'LOW';\n"
        "                valW.className = 'card-value alert';\n"
        "            }\n"
        "\n"
        "            // Pump card\n"
        "            const cardP = document.getElementById('card-pump');\n"
        "            const valP  = document.getElementById('val-pump');\n"
        "            if (d.pump_on) {\n"
        "                cardP.className = 'card active';\n"
        "                valP.textContent = 'ON';\n"
        "                valP.className = 'card-value';\n"
        "            } else {\n"
        "                cardP.className = 'card';\n"
        "                valP.textContent = 'OFF';\n"
        "                valP.className = 'card-value';\n"
        "            }\n"
        "\n"
        "            // Soil card\n"
        "            const soilVal = document.querySelector('#card-pump ~ .card .card-value');\n"
        "            document.querySelectorAll('.status-grid .card')[2]\n"
        "                .querySelector('.card-value').textContent = d.soil_percent + '%';\n"
        "\n"
        "            document.getElementById('val-soil').textContent = d.soil_percent + '%';\n"
        "            document.getElementById('val-temp').textContent  =\n"
        "                d.temp_f.toFixed(1) + '°F';\n"
        "\n"
        "            // Detail rows\n"
        "            document.getElementById('detail-soil').textContent = d.soil_percent + '%';\n"
        "            document.getElementById('detail-hum').textContent  = d.hum_rh.toFixed(1) + '%';\n"
        "            document.getElementById('detail-temp').textContent =\n"
        "                d.temp_f.toFixed(1) + '°F / ' + d.temp_c.toFixed(1) + '°C';\n"
        "            document.getElementById('detail-on-thresh').textContent  = d.pump_on_threshold + '%';\n"
        "            document.getElementById('detail-off-thresh').textContent = d.pump_off_threshold + '%';\n"
        "            document.getElementById('detail-raw').textContent = d.soil_raw;\n"
        "\n"
        "            // Bars\n"
        "            document.getElementById('soil-bar').style.width = d.soil_percent + '%';\n"
        "            document.getElementById('soil-bar').className =\n"
        "                'bar-fill' + (d.soil_percent < d.pump_on_threshold ? ' low' : '');\n"
        "            document.getElementById('hum-bar').style.width = d.hum_rh + '%';\n"
        "\n"
        "            // Pump pill\n"
        "            const pill     = document.getElementById('pump-pill');\n"
        "            const pillText = document.getElementById('pill-text');\n"
        "            if (d.pump_on) {\n"
        "                pill.className = 'pump-pill on';\n"
        "                pillText.textContent = 'RUNNING';\n"
        "            } else {\n"
        "                pill.className = 'pump-pill off';\n"
        "                pillText.textContent = 'IDLE';\n"
        "            }\n"
        "\n"
        "            // Toggle sync (keep toggle in sync with auto-control when not overriding)\n"
        "            if (!manualOverride) {\n"
        "                document.getElementById('pump-toggle').checked = !!d.pump_on;\n"
        "            }\n"
        "            document.getElementById('toggle-label').textContent =\n"
        "                d.pump_on ? 'Pump on' : 'Pump off';\n"
        "\n"
        "            // Override badge\n"
        "            document.getElementById('override-tag').className =\n"
        "                'override-tag' + (manualOverride ? ' visible' : '');\n"
        "\n"
        "            // Timestamp\n"
        "            const ts = document.getElementById('timestamp');\n"
        "            ts.textContent = 'Last updated: ' + new Date().toLocaleTimeString();\n"
        "            ts.className = 'timestamp';\n"
        "            }\n"
        "\n"
        "            // ── Manual pump toggle handler ─────────────────────────────────────────\n"
        "            function handlePumpToggle(checked) {\n"
        "            manualOverride = true;\n"
        "            manualPumpOn   = checked;\n"
        "            mockState.pump_on = checked ? 1 : 0;\n"
        "\n"
        "            // Auto-clear manual override after 30 s (like a real system would)\n"
        "            clearTimeout(window._overrideTimer);\n"
        "            window._overrideTimer = setTimeout(() => {\n"
        "                manualOverride = false;\n"
        "                manualPumpOn   = false;\n"
        "                document.getElementById('override-tag').className = 'override-tag';\n"
        "            }, 30000);\n"
        "\n"
        "            render(mockState);\n"
        "            }\n"
        "\n"
        "            // ── Main loop ──────────────────────────────────────────────────────────\n"
        "            function tick() {\n"
        "            tickMockData();\n"
        "            autoControl(mockState);\n"
        "            render(mockState);\n"
        "            }\n"
        "\n"
        "            // Try to fetch from real ESP32 first; fall back to mock data\n"
        "            function fetchData() {\n"
        "            fetch('/api/status')\n"
        "                .then(r => r.json())\n"
        "                .then(data => {\n"
        "                // Real data available — merge into mockState and render\n"
        "                if (!manualOverride) Object.assign(mockState, data);\n"
        "                render(mockState);\n"
        "                document.querySelector('.mock-banner').style.display = 'none';\n"
        "                })\n"
        "                .catch(() => {\n"
        "                // No ESP32 — use mock simulation\n"
        "                tick();\n"
        "                });\n"
        "            }\n"
        "\n"
        "            // Initial render + interval\n"
        "            tick();\n"
        "            setInterval(fetchData, 2000);\n"
        "        </script>\n"
        "        </body>\n"
        "        </html>\n";

httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    
    return ESP_OK;
}

/**
 * Start HTTP server and register URI handlers
 */
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
            pump_set(1);
            pump_on = 1;
            pump_state_changed = true;
            ESP_LOGI(TAG, "Pump ON (soil moisture %d%% < threshold %d%%)", soil_percent, on_threshold);
            
            // Turn off LCD during pump operation to prevent electrical interference
            if (lcd_display_off() == ESP_OK) {
                lcd_enabled = false;
                ESP_LOGI(TAG, "LCD disabled (pump running)");
            }
            
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
            vTaskDelay(pdMS_TO_TICKS(200));

            // Reset and reactivate LCD with retry logic (pump interference may cause errors)
            bool lcd_reset_success = false;
            for (int retry = 0; retry < 3 && !lcd_reset_success; retry++) {
                if (lcd_clear() == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (lcd_display_on() == ESP_OK) {
                        lcd_enabled = true;
                        lcd_reset_success = true;
                        ESP_LOGI(TAG, "LCD reactivated successfully (attempt %d)", retry + 1);
                    }
                }
                if (!lcd_reset_success && retry < 2) {
                    vTaskDelay(pdMS_TO_TICKS(50));  // Wait before retry
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
