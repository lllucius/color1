/**
 * TCS3530 Color Identification System for ESP-IDF
 * 
 * Features:
 * - Full sensor calibration
 * - Automatic gain and integration time optimization
 * - Color temperature determination
 * - White balance correction
 * - Color matching using octree in 16-bit RGB colorspace
 * - Resene color name identification
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "tcs3530.h"

#define TAG "COLOR_SYSTEM"

/* I2C Configuration */
#define I2C_MASTER_INT_IO 10       // SCL pin
#define I2C_MASTER_SDA_IO 12       // SDA pin
#define I2C_MASTER_SCL_IO 13       // SCL pin
#define I2C_MASTER_PWR_IO 11       // PWR pin
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ 400000  // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS 1000 // I2C timeout

/* Calibration NVS Keys */
#define NVS_NAMESPACE "colorsensor"
#define NVS_CAL_KEY "cal_data"

/* Octree Configuration */
#define MAX_TREE_DEPTH 8  // Max depth for 16-bit RGB (64k colors)

/* Color Tables */
#define MAX_COLOR_TABLE_SIZE 1500

/* Sensor Configuration */
typedef struct {
    uint8_t integration_time;  // ATIME register value
    uint8_t gain;             // CONTROL register gain value
    float cal_red;            // Red calibration factor
    float cal_green;          // Green calibration factor
    float cal_blue;           // Blue calibration factor
    float cal_clear;          // Clear calibration factor
    uint16_t white_ref_r;     // White reference (red)
    uint16_t white_ref_g;     // White reference (green)
    uint16_t white_ref_b;     // White reference (blue)
} tcs3530_config_t;

/* Raw Color Values */
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;  // Clear
} color_raw_t;

/* Normalized Color Values */
typedef struct {
    uint16_t r;  // 16-bit (0-65535)
    uint16_t g;  // 16-bit (0-65535)
    uint16_t b;  // 16-bit (0-65535)
} color_rgb_t;

/* Color with Name */
typedef struct {
    color_rgb_t rgb;
    char name[64];
} named_color_t;

/* Octree Node */
typedef struct octree_node {
    struct octree_node* children[8];
    uint8_t is_leaf;
    uint16_t color_count;
    uint32_t color_sum_r;
    uint32_t color_sum_g;
    uint32_t color_sum_b;
    named_color_t* color;  // Only valid for leaf nodes
} octree_node_t;

// Color space constants
#define CIE_EPSILON               0.008856
#define CIE_KAPPA                 903.3

// Color data structures
typedef struct
{
    float x;
    float y;
    float z;
} XYZ_Color;

typedef struct
{
    float L;
    float a;
    float b;
} LAB_Color;

/* Global Variables */
static tcs3530_config_t sensor_config = {
    .integration_time = 0xEB,  // ~50ms
    .gain = 0x01,             // 4x gain
    .cal_red = 1.0,
    .cal_green = 1.0,
    .cal_blue = 1.0,
    .cal_clear = 1.0,
    .white_ref_r = 65535,
    .white_ref_g = 65535,
    .white_ref_b = 65535,
};

static octree_node_t* color_octree = NULL;
static named_color_t color_table[MAX_COLOR_TABLE_SIZE];
static int color_table_size = 0;

static i2c_master_dev_handle_t i2c_dev_handle = NULL;

/* Function Prototypes */
esp_err_t i2c_master_init(void);
esp_err_t tcs3530_init(void);
esp_err_t tcs3530_set_integration_time(uint8_t time);
esp_err_t tcs3530_set_gain(uint8_t gain);
esp_err_t tcs3530_read_raw_values(color_raw_t* raw_values);
esp_err_t tcs3530_perform_calibration(void);
esp_err_t tcs3530_calibrate_white_balance(void);
esp_err_t tcs3530_optimize_settings(void);
esp_err_t tcs3530_get_color_temperature(uint16_t* temp);
uint8_t get_octree_index(color_rgb_t color, int depth);
octree_node_t* octree_new_node(void);
void octree_insert_color(octree_node_t* root, named_color_t color, int depth);
void octree_build(named_color_t* colors, int count);
named_color_t* octree_find_nearest_color(octree_node_t* root, color_rgb_t color, int depth);
void load_color_table(void);
esp_err_t save_calibration_data(void);
esp_err_t load_calibration_data(void);
color_rgb_t normalize_color(color_raw_t raw, tcs3530_config_t* config);
float color_distance(color_rgb_t a, color_rgb_t b);

/* I2C Helper Functions */
static esp_err_t i2c_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(i2c_dev_handle, write_buf, sizeof(write_buf), 1000);
}

static esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(i2c_dev_handle, &reg_addr, 1, data, len, 1000);
}

/* I2C Master Initialization */
esp_err_t i2c_master_init(void)
{
    esp_err_t ret;

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config =
    {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master bus init failed");
        return ret;
    }

    // Add device to bus
    i2c_device_config_t dev_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS3530_DEVICE_ADDRESS,
        .scl_speed_hz = 40 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C add device failed");
        i2c_del_master_bus(bus_handle);
        return ret;
    }
   
    return ret;
}

#define ESP_INTR_FLAG_DEFAULT 0
static QueueHandle_t gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


/* TCS3530 Initialization */
esp_err_t tcs3530_init(void)
{
    esp_err_t ret;
    uint8_t device_id;
    uint8_t asdf;
    uint8_t qwer[1024];

    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Read device ID to verify connection
    ret = i2c_read_reg(TCS3530_REG_ID, &device_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }
 
    // Verify device ID
    if (device_id != TCS3530_DEVICE_ID)
    {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X", device_id);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "TCS3530 detected with ID: 0x%02X", device_id);

    // Execute a soft reset
    ret = i2c_write_reg(TCS3530_REG_CONTROL_SCL, TCS3530_CONTROL_SCL_SOFT_RESET);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    printf("OSCEN %d: %02x\n", i2c_read_reg(TCS3530_REG_OSCEN, &asdf, 1), asdf);
    printf("ENABLE %d: %02x\n", i2c_read_reg(TCS3530_REG_ENABLE, &asdf, 1), asdf);
    printf("STATUS %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS, &asdf, 1), asdf);
    printf("STATUS2 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS2, &asdf, 1), asdf);
    printf("STATUS3 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS3, &asdf, 1), asdf);
    printf("STATUS4 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS4, &asdf, 1), asdf);
    printf("STATUS5 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS5, &asdf, 1), asdf);
    printf("STATUS6 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS6, &asdf, 1), asdf);

    printf("CFG0 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG0, &asdf, 1), asdf);
    printf("CFG1 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG1, &asdf, 1), asdf);
    printf("CFG2 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG2, &asdf, 1), asdf);
    printf("CFG3 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG3, &asdf, 1), asdf);
    printf("CFG4 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG4, &asdf, 1), asdf);
    printf("CFG5 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG5, &asdf, 1), asdf);
    printf("CFG6 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG6, &asdf, 1), asdf);
    printf("CFG7 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG7, &asdf, 1), asdf);
    printf("CFG8 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG8, &asdf, 1), asdf);
    printf("CFG9 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG9, &asdf, 1), asdf);
 
  
    // Enable the oscillator
    ret = i2c_write_reg(TCS3530_REG_OSCEN, TCS3530_OSCEN_OSCEN);
    if (ret != ESP_OK)
    {
        return ret;
    }
    // Read device ID to verify connection
    ret = i2c_read_reg(TCS3530_REG_OSCEN, &asdf, 1);
    printf("RET %d asdf %d\n", ret, asdf);
    
    // Power on 
    ret = i2c_write_reg(TCS3530_REG_ENABLE, TCS3530_ENABLE_PON);
    if (ret != ESP_OK)
    {
        return ret;
    }
    // Set measurement rate to approx. 200ms
    ret = i2c_write_reg(TCS3530_REG_WTIME, TCS3530_WTIME_DEFAULT);
    if (ret != ESP_OK)
    {
        return ret;
    }
#if 0
    // Enable FIFO interrupt at 16 bytes 
    ret = i2c_write_reg(TCS3530_REG_INTENAB, TCS3530_INTENAB_FIEN);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 
    ret = i2c_write_reg(TCS3530_REG_FIFO_THR, (uint8_t) (TCS3530_FIFO_THR_DEFAULT >> 3));
    if (ret != ESP_OK)
    {
        return ret;
    }


    // 
    ret = i2c_write_reg(TCS3530_REG_CFG2, (uint8_t) (TCS3530_FIFO_THR_DEFAULT & 0x07));
    if (ret != ESP_OK)
    {
        return ret;
    }
#endif
    // Clear the FIFO
    ret = i2c_write_reg(TCS3530_REG_MEAS_MODE0, TCS3530_MEAS_MODE0_MEASUREMENT_SEQUENCER_SINGLE_SHOT_MODE);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Clear the FIFO
    ret = i2c_write_reg(TCS3530_REG_CONTROL, TCS3530_CONTROL_FIFO_CLR);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = i2c_write_reg(TCS3530_REG_VSYNC_GPIO_INT, 0x00);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Power on and enable
    ret = i2c_write_reg(TCS3530_REG_ENABLE, TCS3530_ENABLE_PON | TCS3530_ENABLE_AEN);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}
#if 0
/* Set Integration Time */
esp_err_t tcs3530_set_integration_time(uint8_t time)
{
    esp_err_t ret = i2c_write_reg(TCS3530_ATIME, time);
    if (ret == ESP_OK)
    {
        sensor_config.integration_time = time;
    }
    return ret;
}

/* Set Gain */
esp_err_t tcs3530_set_gain(uint8_t gain) {
    esp_err_t ret = i2c_write_reg(TCS3530_CONTROL, gain);
    if (ret == ESP_OK) {
        sensor_config.gain = gain;
    }
    return ret;
}
#endif

// Define the maximum tristimulus value from the sensor
#define MAX_TRISTIMULUS_VALUE 65535

// Define the maximum XYZ value (assuming a normalized range of 0 to 1 for this example)
#define MAX_XYZ_VALUE 1.0

//#define normalize(v) ((float) (v) / MAX_TRISTIMULUS_VALUE * MAX_XYZ_VALUE)
#define normalize(v) (v)

// Function to compute CCT from XYZ using McCamy's approximation
double compute_CCT(double X, double Y, double Z) {
    double x = X / (X + Y + Z);
    double y = Y / (X + Y + Z);

    // McCamy's approximation for CCT (in Kelvin)
    double n = (x - 0.3320) / (0.1858 - y);
    double CCT = 449.0 * pow(n, 3) + 3525.0 * pow(n, 2) + 6823.3 * n + 5520.33;

    return CCT;
}

/* Read Raw Color Values */
esp_err_t tcs3530_read_data(tcs3530_data_t *data_out)
{
    uint8_t fifo_status[2] = {};
    uint8_t fifo_data[80 * ( 16 + 7)] = {};
    uint16_t fifo_size;
    esp_err_t ret;

    // Get the FIFO status (gets both status 0 and 1)
    ret = i2c_read_reg(TCS3530_REG_FIFO_STATUS0, fifo_status, 2);
    printf("FIFO STATUS %d %02x %02x\n", ret, fifo_status[1], fifo_status[0]);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Ensure we have enough to work with
    fifo_size = ((uint16_t) fifo_status[0] << 3) | (fifo_status[1] & 0x07);
    printf("FIFO SIZE %d\n", fifo_size);
    if (fifo_size < 16)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(fifo_data, 0xff, 1280);
    // Get the queued readings
    ret = i2c_read_reg(TCS3530_REG_FIFO_DATA, fifo_data, fifo_size);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t asdf;
    printf("OSCEN %d: %02x\n", i2c_read_reg(TCS3530_REG_OSCEN, &asdf, 1), asdf);
    printf("ENABLE %d: %02x\n", i2c_read_reg(TCS3530_REG_ENABLE, &asdf, 1), asdf);
    printf("STATUS %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS, &asdf, 1), asdf);
    printf("STATUS2 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS2, &asdf, 1), asdf);
    printf("STATUS3 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS3, &asdf, 1), asdf);
    printf("STATUS4 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS4, &asdf, 1), asdf);
    printf("STATUS5 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS5, &asdf, 1), asdf);
    printf("STATUS6 %d: %02x\n", i2c_read_reg(TCS3530_REG_STATUS6, &asdf, 1), asdf);

    printf("CFG0 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG0, &asdf, 1), asdf);
    printf("CFG1 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG1, &asdf, 1), asdf);
    printf("CFG2 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG2, &asdf, 1), asdf);
    printf("CFG3 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG3, &asdf, 1), asdf);
    printf("CFG4 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG4, &asdf, 1), asdf);
    printf("CFG5 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG5, &asdf, 1), asdf);
    printf("CFG6 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG6, &asdf, 1), asdf);
    printf("CFG7 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG7, &asdf, 1), asdf);
    printf("CFG8 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG8, &asdf, 1), asdf);
    printf("CFG9 %d: %02x\n", i2c_read_reg(TCS3530_REG_CFG9, &asdf, 1), asdf);
 
 
printf("DATA\n");
#if 0
    int ndx = 0;
    for (int i = 0; i < 80; i++)
    {
        printf("%02d: "
            "%02x %02x %02x %02x "
            "%02x %02x %02x %02x "
            "%02x %02x %02x %02x "
            "%02x %02x %02x %02x\n",
            i,
            fifo_data[ndx + 0], fifo_data[ndx + 1], fifo_data[ndx + 2], fifo_data[ndx + 3],
            fifo_data[ndx + 4], fifo_data[ndx + 5], fifo_data[ndx + 6], fifo_data[ndx + 7],
            fifo_data[ndx + 8], fifo_data[ndx + 9], fifo_data[ndx + 10], fifo_data[ndx + 11],
            fifo_data[ndx + 12], fifo_data[ndx + 13], fifo_data[ndx + 14], fifo_data[ndx + 15]);
        ndx += 16;
    }
#endif
    data_out->x = normalize(((uint16_t) fifo_data[1] << 8) | fifo_data[0]);
    data_out->y = normalize(((uint16_t) fifo_data[3] << 8) | fifo_data[2]);
    data_out->z = normalize(((uint16_t) fifo_data[5] << 8) | fifo_data[4]);
    data_out->ir = normalize(((uint16_t) fifo_data[7] << 8) | fifo_data[6]);
    data_out->hgl = normalize(((uint16_t) fifo_data[9] << 8) | fifo_data[8]);
    data_out->hgh = normalize(((uint16_t) fifo_data[11] << 8) | fifo_data[10]);
    data_out->clear = normalize(((uint16_t) fifo_data[13] << 8) | fifo_data[12]);
    data_out->flicker = normalize(((uint16_t) fifo_data[15] << 8) | fifo_data[14]);

    printf("x %d y %d z %d, x %f y %f z %f\n",
        data_out->x, data_out->y, data_out->z,
        (float) data_out->x * (1.0/data_out->clear), (float) data_out->y * (1.0/ data_out->clear), (float) data_out->z * (1.0 / data_out->clear));
    printf("CCT %f\n", compute_CCT(data_out->x, data_out->y, data_out->z));
#if 1 
    // Clear the FIFO
    ret = i2c_write_reg(TCS3530_REG_CONTROL, TCS3530_CONTROL_FIFO_CLR);
    if (ret != ESP_OK)
    {
        return ret;
    }
#endif
    // Get the FIFO status (gets both status 0 and 1)
    ret = i2c_read_reg(TCS3530_REG_FIFO_STATUS0, fifo_status, 2);
    printf("FIFO STATUS %d\n", ret);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Ensure we have enough to work with
    fifo_size = ((uint16_t) fifo_status[0] << 3) | (fifo_status[1] & 0x07);
    printf("FIFO SIZE %d\n", fifo_size);

    return ESP_OK;
}

// Apply white balance and IR correction to raw color values
static XYZ_Color apply_white_balance(XYZ_Color raw_color, uint16_t ir, uint16_t clear)
{
    XYZ_Color corrected;
    
    // Reference white point (D65)
    float white_x = 95.047;
    float white_y = 100.0;
    float white_z = 108.883;
    
    // Simple IR compensation (subtract IR influence from each channel)
    float ir_factor = 0.5;  // Adjust based on empirical testing
    corrected.x = raw_color.x - (ir * ir_factor);
    corrected.y = raw_color.y - (ir * ir_factor);
    corrected.z = raw_color.z - (ir * ir_factor);
    
    // Ensure no negative values
    corrected.x = corrected.x < 0 ? 0 : corrected.x;
    corrected.y = corrected.y < 0 ? 0 : corrected.y;
    corrected.z = corrected.z < 0 ? 0 : corrected.z;
    
    // Scale based on clear channel for normalization
    if ((clear) > 0.0) {
        float scale = 100.0f / (clear);
        corrected.x *= scale;
        corrected.y *= scale;
        corrected.z *= scale;
    }
    
    // White balance correction (scale to D65 white point)
    float max_component = fmaxf(corrected.x, fmaxf(corrected.y, corrected.z));
    if (max_component > 0) {
        float wx = white_x / 100.0f;
        float wy = white_y / 100.0f;
        float wz = white_z / 100.0f;
        
        corrected.x = corrected.x * wx;
        corrected.y = corrected.y * wy;
        corrected.z = corrected.z * wz;
    }
    
    return corrected;
}

// Convert XYZ color to CIELAB
static LAB_Color xyz_to_lab(XYZ_Color xyz)
{
    LAB_Color lab;
    float x_rel, y_rel, z_rel;
    float fx, fy, fz;
    
    // Normalize with reference white point (D65)
    x_rel = xyz.x / 95.047f;
    y_rel = xyz.y / 100.0f;
    z_rel = xyz.z / 108.883f;
    
    // Apply nonlinear compression
    if (x_rel > CIE_EPSILON) {
        fx = powf(x_rel, 1.0f/3.0f);
    } else {
        fx = (CIE_KAPPA * x_rel + 16.0f) / 116.0f;
    }
    
    if (y_rel > CIE_EPSILON) {
        fy = powf(y_rel, 1.0f/3.0f);
    } else {
        fy = (CIE_KAPPA * y_rel + 16.0f) / 116.0f;
    }
    
    if (z_rel > CIE_EPSILON) {
        fz = powf(z_rel, 1.0f/3.0f);
    } else {
        fz = (CIE_KAPPA * z_rel + 16.0f) / 116.0f;
    }
    
    // Calculate L*a*b* values
    lab.L = 116.0f * fy - 16.0f;
    lab.a = 500.0f * (fx - fy);
    lab.b = 200.0f * (fy - fz);
    
    // Ensure valid ranges
    lab.L = fmaxf(0.0f, fminf(100.0f, lab.L));
    
    return lab;
}

/* Application Example */
void app_main(void) {
    esp_err_t ret;
    char color_name[64];
    uint16_t color_temp;
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase and reinitialize NVS
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C master
    ESP_LOGI(TAG, "Initializing I2C master...");
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
#if 0
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<I2C_MASTER_INT_IO);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(I2C_MASTER_INT_IO, gpio_isr_handler, (void*) I2C_MASTER_INT_IO);
#endif
    // Initialize TCS3530 sensor
    ESP_LOGI(TAG, "Initializing TCS3530 sensor...");
    ret = tcs3530_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCS3530 initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    // Clear the FIFO
    i2c_write_reg(TCS3530_REG_CONTROL, TCS3530_CONTROL_FIFO_CLR);

    // Main loop for color identification
    while (1)
    { 
    ESP_LOGI(TAG, "Starting color identification loop...");
        uint32_t io_num = 0;
        if (xQueueReceive(gpio_evt_queue, &io_num, 0)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }

        tcs3530_data_t data;
        ret = tcs3530_read_data(&data);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "X: %u\r\n", data.x );
            ESP_LOGI(TAG, "Y: %u\r\n", data.y );
            ESP_LOGI(TAG, "Z: %u\r\n", data.z );
            ESP_LOGI(TAG, "IR: %u\r\n", data.ir );
            ESP_LOGI(TAG, "HgL: %u\r\n", data.hgl );
            ESP_LOGI(TAG, "HgH: %u\r\n", data.hgh );
            ESP_LOGI(TAG, "Clear: %u\r\n", data.clear );
            ESP_LOGI(TAG, "Flicker: %u\r\n\n", data.flicker );

            // Convert to float
            XYZ_Color raw_color =
            {
                data.x, // / 65535.0,
                data.y, // / 65535.0,
                data.z, // / 65535.0
            };

            ESP_LOGI(TAG, "RAW X: %f\r\n", raw_color.x );
            ESP_LOGI(TAG, "RAW Y: %f\r\n", raw_color.y );
            ESP_LOGI(TAG, "RAW Z: %f\r\n", raw_color.z );

            // Apply white balance and IR correction
            XYZ_Color color = apply_white_balance(raw_color, data.ir, data.clear);

            ESP_LOGI(TAG, "CORRECT X: %f\r\n", color.x );
            ESP_LOGI(TAG, "CORRECT Y: %f\r\n", color.y );
            ESP_LOGI(TAG, "CORRECT Z: %f\r\n", color.z );

        } else {
            ESP_LOGE(TAG, "Failed to get color temperature: %s", esp_err_to_name(ret));
        }
        
        // Wait before next measurement
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }

#if 0    
    // Load the color table and build octree
    ESP_LOGI(TAG, "Loading color table...");
    load_color_table();
    
    // Perform full calibration (uncomment to run during initialization)
    // ESP_LOGI(TAG, "Starting sensor calibration...");
    // ret = tcs3530_perform_calibration();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(ret));
    // }
    
    // Determine optimal gain and integration time
    ESP_LOGI(TAG, "Optimizing sensor settings...");
    ret = tcs3530_optimize_settings();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting optimization failed: %s", esp_err_to_name(ret));
    }

    // Main loop for color identification
    ESP_LOGI(TAG, "Starting color identification loop...");
    while (1) {
        // Get color temperature
        ret = tcs3530_get_color_temperature(&color_temp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Color temperature: %dK", color_temp);
        } else {
            ESP_LOGE(TAG, "Failed to get color temperature: %s", esp_err_to_name(ret));
        }
        
        // Identify color
        ret = identify_color(color_name, sizeof(color_name));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Identified color: %s", color_name);
        } else {
            ESP_LOGE(TAG, "Color identification failed: %s", esp_err_to_name(ret));
        }
        
        // Wait before next measurement
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif
}

/* Extended Color Table Implementation */
// In a real application, a complete color table would be loaded from flash storage
// or embedded data. Below is an implementation to load a larger table from a header file.

#ifdef INCLUDE_FULL_RESENE_TABLE
// This would typically be in a separate header file
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    const char* name;
} resene_color_entry_t;

// Sample of Resene color table (full table would have hundreds of entries)
const resene_color_entry_t full_resene_table[] = {
    {65535, 64250, 61166, "Resene Alabaster"},
    {58082, 44975, 44975, "Resene Cavern Pink"},
    {26985, 18504, 18247, "Resene Bulgari"},
    // ...many more entries...
};

const int full_resene_table_size = sizeof(full_resene_table) / sizeof(full_resene_table[0]);

// Function to load the full color table
void load_full_color_table(void) {
    int count = (full_resene_table_size < MAX_COLOR_TABLE_SIZE) ? 
                 full_resene_table_size : MAX_COLOR_TABLE_SIZE;
    
    for (int i = 0; i < count; i++) {
        color_table[i].rgb.r = full_resene_table[i].r;
        color_table[i].rgb.g = full_resene_table[i].g;
        color_table[i].rgb.b = full_resene_table[i].b;
        strncpy(color_table[i].name, full_resene_table[i].name, sizeof(color_table[i].name) - 1);
        color_table[i].name[sizeof(color_table[i].name) - 1] = '\0';
    }
    
    color_table_size = count;
    ESP_LOGI(TAG, "Loaded full color table with %d colors", color_table_size);
    
    // Build octree with the loaded colors
    octree_build(color_table, color_table_size);
}
#endif

/* Implementation of a Better Color Distance Metric: CIEDE2000 */
// This provides a more perceptually accurate color comparison than Euclidean distance
// Simplified implementation for embedded systems

typedef struct {
    double L;  // Lightness
    double a;  // Green-Red
    double b;  // Blue-Yellow
} lab_color_t;

// Convert RGB to Lab colorspace
lab_color_t rgb_to_lab(color_rgb_t rgb) {
    lab_color_t lab;
    
    // Normalize RGB to 0-1
    double r = rgb.r / 65535.0;
    double g = rgb.g / 65535.0;
    double b = rgb.b / 65535.0;
    
    // Convert to sRGB
    r = (r > 0.04045) ? pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
    g = (g > 0.04045) ? pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
    b = (b > 0.04045) ? pow((b + 0.055) / 1.055, 2.4) : b / 12.92;
    
    // Convert to XYZ
    double X = r * 0.4124 + g * 0.3576 + b * 0.1805;
    double Y = r * 0.2126 + g * 0.7152 + b * 0.0722;
    double Z = r * 0.0193 + g * 0.1192 + b * 0.9505;
    
    // XYZ to Lab
    // Reference values for D65 illuminant
    const double Xn = 0.95047;
    const double Yn = 1.0;
    const double Zn = 1.08883;
    
    X /= Xn;
    Y /= Yn;
    Z /= Zn;
    
    X = (X > 0.008856) ? pow(X, 1.0/3.0) : (7.787 * X) + (16.0 / 116.0);
    Y = (Y > 0.008856) ? pow(Y, 1.0/3.0) : (7.787 * Y) + (16.0 / 116.0);
    Z = (Z > 0.008856) ? pow(Z, 1.0/3.0) : (7.787 * Z) + (16.0 / 116.0);
    
    lab.L = (116.0 * Y) - 16.0;
    lab.a = 500.0 * (X - Y);
    lab.b = 200.0 * (Y - Z);
    
    return lab;
}

// Calculate CIEDE2000 color difference
double ciede2000_distance(color_rgb_t rgb1, color_rgb_t rgb2) {
    // Convert RGB to Lab
    lab_color_t lab1 = rgb_to_lab(rgb1);
    lab_color_t lab2 = rgb_to_lab(rgb2);
    
    // Parameters
    double kL = 1.0;
    double kC = 1.0;
    double kH = 1.0;
    
    // Calculate C1, C2, C_avg
    double C1 = sqrt(lab1.a * lab1.a + lab1.b * lab1.b);
    double C2 = sqrt(lab2.a * lab2.a + lab2.b * lab2.b);
    double C_avg = (C1 + C2) / 2.0;
    
    // Calculate a' values
    double G = 0.5 * (1.0 - sqrt(pow(C_avg, 7) / (pow(C_avg, 7) + pow(25.0, 7))));
    double a1_prime = lab1.a * (1.0 + G);
    double a2_prime = lab2.a * (1.0 + G);
    
    // Calculate C' values
    double C1_prime = sqrt(a1_prime * a1_prime + lab1.b * lab1.b);
    double C2_prime = sqrt(a2_prime * a2_prime + lab2.b * lab2.b);
    double C_avg_prime = (C1_prime + C2_prime) / 2.0;
    
    // Calculate h' values
    double h1_prime = atan2(lab1.b, a1_prime);
    if (h1_prime < 0) h1_prime += 2.0 * M_PI;
    
    double h2_prime = atan2(lab2.b, a2_prime);
    if (h2_prime < 0) h2_prime += 2.0 * M_PI;
    
    // Calculate ΔL', ΔC', ΔH'
    double delta_L_prime = lab2.L - lab1.L;
    double delta_C_prime = C2_prime - C1_prime;
    
    double delta_h_prime;
    if (C1_prime * C2_prime == 0) {
        delta_h_prime = 0;
    } else {
        if (fabs(h2_prime - h1_prime) <= M_PI) {
            delta_h_prime = h2_prime - h1_prime;
        } else if (h2_prime - h1_prime > M_PI) {
            delta_h_prime = h2_prime - h1_prime - 2.0 * M_PI;
        } else {
            delta_h_prime = h2_prime - h1_prime + 2.0 * M_PI;
        }
    }
    
    double delta_H_prime = 2.0 * sqrt(C1_prime * C2_prime) * sin(delta_h_prime / 2.0);
    
    // Lightness weight
    double L_avg = (lab1.L + lab2.L) / 2.0;
    double SL = 1.0 + (0.015 * pow(L_avg - 50.0, 2)) / sqrt(20.0 + pow(L_avg - 50.0, 2));
    
    // Chroma weight
    double SC = 1.0 + 0.045 * C_avg_prime;
    
    // Hue weight
    double T = 1.0 - 0.17 * cos(h1_prime - M_PI/6.0) + 0.24 * cos(2.0 * h1_prime) +
               0.32 * cos(3.0 * h1_prime + M_PI/30.0) - 0.20 * cos(4.0 * h1_prime - 63.0 * M_PI/180.0);
    double SH = 1.0 + 0.015 * C_avg_prime * T;
    
    // Rotation factor
    double h_avg_prime;
    if (fabs(h1_prime - h2_prime) <= M_PI) {
        h_avg_prime = (h1_prime + h2_prime) / 2.0;
    } else if (h1_prime + h2_prime < 2.0 * M_PI) {
        h_avg_prime = (h1_prime + h2_prime + 2.0 * M_PI) / 2.0;
    } else {
        h_avg_prime = (h1_prime + h2_prime - 2.0 * M_PI) / 2.0;
    }
    
    double RT = -2.0 * sqrt(pow(C_avg_prime, 7) / (pow(C_avg_prime, 7) + pow(25.0, 7))) *
                sin(60.0 * M_PI/180.0 * exp(-pow((h_avg_prime - 275.0 * M_PI/180.0) / (25.0 * M_PI/180.0), 2)));
    
    // Calculate CIEDE2000 color difference
    double delta_E = sqrt(pow(delta_L_prime / (kL * SL), 2) +
                          pow(delta_C_prime / (kC * SC), 2) +
                          pow(delta_H_prime / (kH * SH), 2) +
                          RT * (delta_C_prime / (kC * SC)) * (delta_H_prime / (kH * SH)));
                          
    return delta_E;
}

/* Additional Utility for Debugging and Testing */

// Print detailed color information for debugging
void print_color_details(color_raw_t raw, color_rgb_t rgb, const char* color_name, uint16_t temp) {
    ESP_LOGI(TAG, "------ Color Details ------");
    ESP_LOGI(TAG, "Raw Sensor Values:");
    ESP_LOGI(TAG, "  Red:   %d", raw.r);
    ESP_LOGI(TAG, "  Green: %d", raw.g);
    ESP_LOGI(TAG, "  Blue:  %d", raw.b);
    ESP_LOGI(TAG, "  Clear: %d", raw.c);
    ESP_LOGI(TAG, "Normalized RGB (16-bit):");
    ESP_LOGI(TAG, "  Red:   %d (%.2f%%)", rgb.r, (rgb.r * 100.0) / 65535.0);
    ESP_LOGI(TAG, "  Green: %d (%.2f%%)", rgb.g, (rgb.g * 100.0) / 65535.0);
    ESP_LOGI(TAG, "  Blue:  %d (%.2f%%)", rgb.b, (rgb.b * 100.0) / 65535.0);
    ESP_LOGI(TAG, "Color Temperature: %dK", temp);
    ESP_LOGI(TAG, "Identified Color: %s", color_name);
    ESP_LOGI(TAG, "---------------------------");
}

// Function to test octree lookup
void test_octree(void) {
    // Define test colors
    color_rgb_t test_colors[] = {
        {65535, 0, 0},      // Pure red
        {0, 65535, 0},      // Pure green
        {0, 0, 65535},      // Pure blue
        {65535, 65535, 0},  // Yellow
        {65535, 0, 65535},  // Magenta
        {0, 65535, 65535},  // Cyan
        {32768, 32768, 32768}, // Gray
        {65535, 32768, 0},  // Orange
        {32768, 0, 32768},  // Purple
    };
    
    ESP_LOGI(TAG, "Testing octree lookup with %d colors...", 
              sizeof(test_colors) / sizeof(test_colors[0]));
    
    for (int i = 0; i < sizeof(test_colors) / sizeof(test_colors[0]); i++) {
        named_color_t* result = octree_find_nearest_color(color_octree, test_colors[i], 0);
        
        if (result != NULL) {
            ESP_LOGI(TAG, "Test color %d: Matched to %s", i, result->name);
        } else {
            ESP_LOGE(TAG, "Test color %d: No match found", i);
        }
    }
}
