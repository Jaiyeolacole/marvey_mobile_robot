#include "color_sensor.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "color_sensor";

// ---------------- I2C Bus Config ----------------
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     100000
#define I2C_TIMEOUT_MS  100

// ---------------- TCS34725 Registers ----------------
#define TCS34725_ADDRESS      0x29
#define TCS34725_COMMAND_BIT  0x80

#define TCS34725_ENABLE       0x00
#define TCS34725_ENABLE_PON   0x01
#define TCS34725_ENABLE_AEN   0x02
#define TCS34725_ATIME        0x01
#define TCS34725_CONTROL      0x0F
#define TCS34725_ID           0x12
#define TCS34725_CDATAL       0x14
#define TCS34725_RDATAL       0x16
#define TCS34725_GDATAL       0x18
#define TCS34725_BDATAL       0x1A

static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL;

// ---------------- Low-level I2C helpers ----------------
static esp_err_t tcs_write8(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { (uint8_t)(TCS34725_COMMAND_BIT | reg), value };
    return i2c_master_transmit(s_dev_handle, buf, sizeof(buf),
                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t tcs_read8(uint8_t reg, uint8_t *value)
{
    uint8_t reg_addr = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    return i2c_master_transmit_receive(s_dev_handle, &reg_addr, 1, value, 1,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t tcs_read16(uint8_t reg, uint16_t *value)
{
    uint8_t reg_addr = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(s_dev_handle, &reg_addr, 1, data, 2,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err == ESP_OK) {
        *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    }
    return err;
}

esp_err_t color_sensor_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS34725_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCS34725 device: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t id = 0;
    err = tcs_read8(TCS34725_ID, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read TCS34725 ID register - check wiring.");
        return err;
    }
    ESP_LOGI(TAG, "TCS34725 ID: 0x%02X (expect 0x44 or 0x4D)", id);

    tcs_write8(TCS34725_ATIME, 0xD5);   // true ~101ms integration time (0xEB was actually ~50ms)
    tcs_write8(TCS34725_CONTROL, 0x02); // 16x gain (was 4x) - more signal in low light

    tcs_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    vTaskDelay(pdMS_TO_TICKS(3));
    tcs_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    vTaskDelay(pdMS_TO_TICKS(120)); // wait for first valid integration cycle

    ESP_LOGI(TAG, "Color sensor initialized.");
    return ESP_OK;
}

esp_err_t color_sensor_get_raw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    esp_err_t err;
    err = tcs_read16(TCS34725_CDATAL, c); if (err != ESP_OK) return err;
    err = tcs_read16(TCS34725_RDATAL, r); if (err != ESP_OK) return err;
    err = tcs_read16(TCS34725_GDATAL, g); if (err != ESP_OK) return err;
    err = tcs_read16(TCS34725_BDATAL, b); if (err != ESP_OK) return err;
    return ESP_OK;
}

// NOTE: these thresholds are a starting point, not exact for your
// sensor/lighting. Calibrate by enabling the debug log below,
// placing the sensor over each color (and the black line, and
// bare floor), and reading the raw r/g/b/c values via `idf.py
// monitor`. Adjust the percentage thresholds to match.
color_id_t color_classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    // ESP_LOGI(TAG, "R:%d G:%d B:%d C:%d", r, g, b, c); // uncomment to calibrate

    if (c < 300) {
        return COLOR_BLACK; // low reflectance -> line or dark surface
    }

    float total = (float)(r + g + b);
    if (total < 1.0f) total = 1.0f;
    float rp = r / total, gp = g / total, bp = b / total;

    if (rp > 0.42f && gp < 0.35f && bp < 0.35f) return COLOR_RED;
    if (gp > 0.40f && rp < 0.38f && bp < 0.35f) return COLOR_GREEN;
    if (bp > 0.40f && rp < 0.35f && gp < 0.38f) return COLOR_BLUE;
    if (rp > 0.33f && gp > 0.33f && bp < 0.28f) return COLOR_YELLOW;

    return COLOR_NONE; // e.g. plain white/gray floor
}

const char *color_name(color_id_t color)
{
    switch (color) {
        case COLOR_RED:    return "RED";
        case COLOR_GREEN:  return "GREEN";
        case COLOR_BLUE:   return "BLUE";
        case COLOR_YELLOW: return "YELLOW";
        case COLOR_BLACK:  return "BLACK";
        default:           return "NONE";
    }
}
