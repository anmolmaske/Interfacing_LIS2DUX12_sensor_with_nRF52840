/**
* \file main.c
*
* \brief LIS2DUX12 Sensor Interfacing
*
* \date 2024-12-06
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include "lis2dux12_reg.h"

#include "switch_monitoring.h"
#include "vibration_monitoring.h"

LOG_MODULE_REGISTER(lis_sensor);

/* Self-test Threshold Values */
#define SELF_TEST_MIN_THRESHOLD -5000.0f
#define SELF_TEST_MAX_THRESHOLD  5000.0f

double lis_x = 0.0;
double lis_y = 0.0;
double lis_z = 0.0;

float low_gx = 0.0f;
float low_gy = 0.0f;
float low_gz = 0.0f;

float avg_x = 0.0f;
float avg_y = 0.0f;
float avg_z = 0.0f;


/* User-defined threshold in mg */
uint16_t motion_threshold_mg = 5;

/* Motion Detection Flag */
bool motion_detected = false;

/* I2C Device */
#define I2C_DEV DT_NODELABEL(i2c0)  // To get i2c LIS sensor
static const struct device *i2c_dev;

#define INT1_PORT DT_NODELABEL(gpio0) // To get interrupt pin of LIS sensor 
const struct device *gpio_dev;


/* GPIO Pin for LIS2DUX12 Interrupt (INT1 connected to GPIO 16) */
#define INT1_PIN 16         // Pin 16
#define INT1_FLAGS GPIO_ACTIVE_HIGH

/* GPIO Callback Structure */
static struct gpio_callback motion_cb_data;

/* LIS2DUX12 Context */
stmdev_ctx_t lis_ctx;
uint8_t i2c_address = 0x19; // Address of LIS2DUX12 sensor


/* Custom I2C Read/Write Functions */
static int32_t lis2dux12_i2c_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    const struct device *i2c_dev = (const struct device *)handle;
    int ret = i2c_burst_write(i2c_dev, i2c_address, reg, data, len);
    if (ret < 0) {
        LOG_ERR("I2C Write Error: %d", ret);
    }
    return ret;
}

static int32_t lis2dux12_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    const struct device *i2c_dev = (const struct device *)handle;
    int ret = i2c_burst_read(i2c_dev, i2c_address, reg, data, len);
    if (ret < 0) {
        LOG_ERR("I2C Read Error: %d", ret);
    }
    return ret;
}


/* Initialize the LIS2DUX12 */
void lis2dux12_init(void) {

    i2c_dev = DEVICE_DT_GET(I2C_DEV);
    if (!device_is_ready(i2c_dev)) {
        LOG_INF("LIS2DUX12 I2C device not ready\n");
        return;
    }

    gpio_dev = DEVICE_DT_GET(INT1_PORT);
    if (!device_is_ready(gpio_dev)) {
        LOG_INF("Error: Failed to bind INT1 pin to GPIO device.\n");
        return;
    }

    uint8_t who_am_i;
    int ret;

    lis_ctx.write_reg = lis2dux12_i2c_write;
    lis_ctx.read_reg = lis2dux12_i2c_read;
    lis_ctx.handle = (void *)i2c_dev;

    /* reset device */
	ret = lis2dux12_init_set(&lis_ctx, LIS2DUX12_RESET);
	if (ret < 0) {
		return ret;
	}
    k_msleep(50);

    /* Read WHO_AM_I Register */
    if ((lis2dux12_device_id_get(&lis_ctx, &who_am_i)) || (who_am_i != LIS2DUX12_ID)) {
        LOG_ERR("Failed to detect LIS2DUX12 (WHO_AM_I = 0x%02X)", who_am_i);
        return;
    } else {
        LOG_INF("LIS2DUX12 detected (WHO_AM_I = 0x%02X)", who_am_i);
    }

    /* Configure Sensor */
    ret = lis2dux12_init_set(&lis_ctx, LIS2DUX12_SENSOR_ONLY_ON);
    if (ret != 0) {
        LOG_ERR("Failed to configure LIS2DUX12");
        return;
    }

    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,
        .fs = LIS2DUX12_4g,
        .bw = LIS2DUX12_ODR_div_4,
    };
    ret = lis2dux12_mode_set(&lis_ctx, &mode);
    if (ret != 0) {
        LOG_INF("Error: Failed to configure LIS2DUX12 mode.\n");
        return;
    }

    LOG_INF("LIS2DUX12 initialized successfully\n");
}

/* If Initilization failed than reinitilize it */
static void lis2dux12_init_with_retries(void) {
    uint8_t device_id;
    int ret;
    int retries = 5;

    /* Assign the I2C read/write functions to the context */
    lis_ctx.write_reg = lis2dux12_i2c_write;
    lis_ctx.read_reg = lis2dux12_i2c_read;
    lis_ctx.handle = (void *)i2c_dev;

    for (int i = 0; i < retries; i++) {
        ret = lis2dux12_device_id_get(&lis_ctx, &device_id);
        if (ret == 0 && device_id == LIS2DUX12_ID) {
            LOG_INF("LIS2DUX12 Device ID: 0x%02X", device_id);
            break;
        }

        LOG_WRN("Retrying LIS2DUX12 initialization (%d/%d)...", i + 1, retries);
        k_msleep(100); // Wait before retrying
    }

    if (ret != 0 || device_id != LIS2DUX12_ID) {
        LOG_ERR("Failed to initialize LIS2DUX12 after %d retries", retries);
    } else {
        LOG_INF("LIS2DUX12 initialized successfully.");

        lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,
        .fs = LIS2DUX12_4g,
        .bw = LIS2DUX12_ODR_div_4,
        };
        ret = lis2dux12_mode_set(&lis_ctx, &mode);
        if (ret != 0) {
            LOG_INF("Error: Failed to configure LIS2DUX12 mode.\n");
            return;
        }
    }
}


/* Calculate inclination angle */
static float get_inclination_angle(const float x, const float y, const float z, uint8_t ref_axis) {
    float inclination_angle = 0.0f;

    switch (ref_axis) {
    case 0:  // X-axis
        inclination_angle = atan((x) / sqrt(((y) * (y)) + ((z) * (z))));
        break;
    case 1:  // Y-axis
        inclination_angle = atan((y) / sqrt(((x) * (x)) + ((z) * (z))));
        break;
    case 2:  // Z-axis
        inclination_angle = atan(sqrt(((y) * (y)) + ((x) * (x))) / (z));
        break;
    default:
        return 0.0f;
    }

    // Convert the angle from radians to degrees
    inclination_angle = (inclination_angle * 180.0 / 3.14159);
    return inclination_angle;
}

/* Fetch Sensor Data */
void fetch_lis_sensor_values(void) {
    lis2dux12_xl_data_t data;
    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,
        .fs = LIS2DUX12_4g,
        .bw = LIS2DUX12_ODR_div_4,
    };

    int ret = lis2dux12_xl_data_get(&lis_ctx, &mode, &data);
    if (ret != 0) {
        LOG_INF("Failed to fetch data from LIS2DUX12\n");
        return;
    }

    /* Store Acceleration Values in Separate Variables */
    lis_x = data.mg[0] / 1000.0 * 9.8;
    lis_y = data.mg[1] / 1000.0 * 9.8;
    lis_z = data.mg[2] / 1000.0 * 9.8;

    /* Acceleration Values in mg */
    // printk("Acceleration [mg]: X=%.2f, Y=%.2f, Z=%.2f\n", data.mg[0], data.mg[1], data.mg[2]);

    /* Acceleration Values in m/s^2 */
    // printk("Acceleration [m/s^2]: X=%.2f, Y=%.2f, Z=%.2f\n", lis_x, lis_y, lis_z);

    /* Calculate Inclination Angles */
    low_gx = get_inclination_angle(lis_x, lis_y, lis_z, 0);
    low_gy = get_inclination_angle(lis_x, lis_y, lis_z, 1);
    low_gz = get_inclination_angle(lis_x, lis_y, lis_z, 2);

    /* Log Inclination Angles */
    // printk("Inclination Angles [Degrees]: X=%.2f, Y=%.2f, Z=%.2f\n", low_gx, low_gy, low_gz);
}


/* Function to read and process LIS2DUX12 sensor angle data */
void read_lis2dux12_angle(uint8_t angle_selection_mode) {
    fetch_lis_sensor_values();

    /* Inclination angle calculation */
    if (angle_selection_mode == 0) {
        // LOG_INF("Angle Selection Mode %d is sleep", angle_selection_mode);
    } else if (angle_selection_mode == 1) {
        LOG_INF("Angles [Degrees]: X = %.2f", low_gx);
    } else if (angle_selection_mode == 2) {
        LOG_INF("Angles [Degrees]: Y = %.2f", low_gy);
    } else if (angle_selection_mode == 4) {
        LOG_INF("Angles [Degrees]: X = %.2f, Y = %.2f", low_gx, low_gy);
    } else if (angle_selection_mode == 3) {
        LOG_INF("Angles [Degrees]: Z = %.2f", low_gz);
    } else if (angle_selection_mode == 5) {
        LOG_INF("Angles [Degrees]: X = %.2f, Z = %.2f", low_gx, low_gz);
    } else if (angle_selection_mode == 6) {
        LOG_INF("Angles [Degrees]: Y = %.2f, Z = %.2f", low_gy, low_gz);
    } else if (angle_selection_mode == 7) {
        LOG_INF("Angles [Degrees]: X = %.2f, Y = %.2f, Z = %.2f", low_gx, low_gy, low_gz);
    } else {
        LOG_INF("Angle Selection Mode %d is invalid", angle_selection_mode);
    }
}

/* Function to read and process LIS2DUX12 sensor Avg Acceleration data */
void read_lis2dux12_avg_accel(uint8_t avg_accel_selection_mode) {

    fetch_lis_sensor_values();

    static double total_x = 0.0;
    static double total_y = 0.0;
    static double total_z = 0.0;
    static int sample_count = 1;
    
    for(int i=0; i<52; i++){

        // adding the acceleration total  value of the x,y,z accels into this variables.
        total_x = lis_x + total_x;
        total_y = lis_y + total_y;
        total_z = lis_z + total_z;

        //checking the count 
        sample_count +=1;
    }

    /* Average acceleration calculation */
    if (avg_accel_selection_mode == 0) {
        // LOG_INF("Avg Acceleration Selection Mode %d is sleep", avg_accel_selection_mode);
    } else if (avg_accel_selection_mode == 1) {
        avg_x = (float)(total_x / sample_count);
        LOG_INF("Avg Accel [m/s^2]: X = %.2f", avg_x);
    } else if (avg_accel_selection_mode == 2) {
        avg_y = (float)(total_y / sample_count);
        LOG_INF("Avg Accel [m/s^2]: Y = %.2f", avg_y);
    } else if (avg_accel_selection_mode == 3) {
        avg_x = (float)(total_x / sample_count);
        avg_y = (float)(total_y / sample_count);
        LOG_INF("Avg Accel [m/s^2]: X = %.2f, Y = %.2f", avg_x, avg_y);
    } else if (avg_accel_selection_mode == 4) {
        avg_z = (float)(total_z / sample_count);
        LOG_INF("Avg Accel [m/s^2]: Z = %.2f", avg_z);
    } else if (avg_accel_selection_mode == 5) {
        avg_x = (float)(total_x / sample_count);
        avg_z = (float)(total_z / sample_count);
        LOG_INF("Avg Accel [m/s^2]: X = %.2f, Z = %.2f", avg_x, avg_z);
    } else if (avg_accel_selection_mode == 6) {
        avg_y = (float)(total_y / sample_count);
        avg_z = (float)(total_z / sample_count);
        LOG_INF("Avg Accel [m/s^2]: Y = %.2f, Z = %.2f", avg_y, avg_z);
    } else if (avg_accel_selection_mode == 7) {
        avg_x = (float)(total_x / sample_count);
        avg_y = (float)(total_y / sample_count);
        avg_z = (float)(total_z / sample_count);
        LOG_INF("Avg Accel [m/s^2]: X = %.2f, Y = %.2f, Z = %.2f", avg_x, avg_y, avg_z);
    } else {
        LOG_INF("Avg Acceleration Selection Mode %d is invalid", avg_accel_selection_mode);
    }
}


/* Perform LIS2DUX12 Self-Test */
bool lis2dux12_self_test(stmdev_ctx_t *ctx) {
    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,
        .fs = LIS2DUX12_4g,
        .bw = LIS2DUX12_ODR_div_4,
    };

    lis2dux12_xl_data_t data;
    float axis_reading[3];
    bool test_passed = true;
    int ret;

    /* Step 1: Configure Sensor Mode */
    ret = lis2dux12_mode_set(ctx, &mode);
    if (ret != 0) {
        LOG_INF("Error: Failed to configure LIS2DUX12 mode\n");
        return false;
    }

    /* Step 2: Positive Self-Test */
    LOG_INF("Starting Positive Self-Test...\n");
    ret = lis2dux12_self_test_sign_set(ctx, LIS2DUX12_XL_ST_POSITIVE);
    if (ret != 0) {
        LOG_INF("Error: Failed to configure positive self-test sign\n");
        return false;
    }

    ret = lis2dux12_self_test_start(ctx, 2);  // Start self-test
    if (ret != 0) {
        LOG_INF("Error: Failed to start positive self-test\n");
        return false;
    }

    k_sleep(K_MSEC(100));  // Wait for stabilization

    /* Read Acceleration Data */
    ret = lis2dux12_xl_data_get(ctx, &mode, &data);
    if (ret != 0) {
        LOG_INF("Error: Failed to fetch positive self-test data\n");
        lis2dux12_self_test_stop(ctx);
        return false;
    }

    axis_reading[0] = data.mg[0];
    axis_reading[1] = data.mg[1];
    axis_reading[2] = data.mg[2];

    LOG_INF("Positive Self-Test Readings [mg]: X=%.2f, Y=%.2f, Z=%.2f\n",
           axis_reading[0], axis_reading[1], axis_reading[2]);

    /* Check Against Thresholds */
    for (int i = 0; i < 3; i++) {
        if (axis_reading[i] < SELF_TEST_MIN_THRESHOLD || axis_reading[i] > SELF_TEST_MAX_THRESHOLD) {
            test_passed = false;
            LOG_INF("Positive Self-Test Failed on Axis %d: %.2f mg\n", i, axis_reading[i]);
        }
    }

    /* Step 3: Negative Self-Test */
    LOG_INF("Starting Negative Self-Test...\n");
    ret = lis2dux12_self_test_sign_set(ctx, LIS2DUX12_XL_ST_NEGATIVE);
    if (ret != 0) {
        LOG_INF("Error: Failed to configure negative self-test sign\n");
        return false;
    }

    ret = lis2dux12_self_test_start(ctx, 2);  // Start self-test
    if (ret != 0) {
        LOG_INF("Error: Failed to start negative self-test\n");
        return false;
    }

    k_sleep(K_MSEC(100));  // Wait for stabilization

    /* Read Acceleration Data */
    ret = lis2dux12_xl_data_get(ctx, &mode, &data);
    if (ret != 0) {
        LOG_INF("Error: Failed to fetch negative self-test data\n");
        lis2dux12_self_test_stop(ctx);
        return false;
    }

    axis_reading[0] = data.mg[0];
    axis_reading[1] = data.mg[1];
    axis_reading[2] = data.mg[2];

    LOG_INF("Negative Self-Test Readings [mg]: X=%.2f, Y=%.2f, Z=%.2f\n",
           axis_reading[0], axis_reading[1], axis_reading[2]);

    /* Check Against Thresholds */
    for (int i = 0; i < 3; i++) {
        if (axis_reading[i] < SELF_TEST_MIN_THRESHOLD || axis_reading[i] > SELF_TEST_MAX_THRESHOLD) {
            test_passed = false;
            LOG_INF("Negative Self-Test Failed on Axis %d: %.2f mg\n", i, axis_reading[i]);
        }
    }

    /* Step 4: Stop Self-Test */
    ret = lis2dux12_self_test_stop(ctx);
    if (ret != 0) {
        LOG_INF("Error: Failed to stop self-test\n");
    }

    return test_passed;
}

/* Start Self-Test of Sensor */
void perform_self_test(void) {
    LOG_INF("Performing LIS2DUX12 Self-Test...\n");
    if (lis2dux12_self_test(&lis_ctx)) {
        LOG_INF("LIS2DUX12 Self-Test PASSED!\n");
    } else {
        LOG_INF("LIS2DUX12 Self-Test FAILED!\n");
    }
}


/* I2C Power-Up Sequence */
int lis2dux12_i2c_power_up_sequence(stmdev_ctx_t *ctx) {
    uint8_t dummy_data = 0x00;
    int ret;

    /* Step 1: Send the STATIC ADDRESS with a Write command */
    ret = lis2dux12_write_reg(ctx, 0x00, &dummy_data, 1);  // Dummy write to trigger power-up
    if (ret == -EIO) {
        /* NACK is expected during power-up */
        LOG_INF("NACK received. Power-up sequence initiated.\n");
    } else if (ret == 0) {
        /* Unexpected ACK */
        LOG_INF("Error: Unexpected ACK during power-up. Device may already be active.\n");
        return -1;
    } else {
        /* Other I2C errors */
        LOG_INF("Error: Failed to send power-up command (I2C error: %d).\n", ret);
        return -1;
    }

    /* Step 2: Wait for the power-up process to complete */
    k_sleep(K_MSEC(25));  // Wait for the sensor to transition to SOFT_PD

    /* Step 3: Verify that the device has transitioned to SOFT_PD */
    ret = lis2dux12_write_reg(ctx, 0x00, &dummy_data, 1);
    if (ret != 0) {
        LOG_INF("Error: Device did not respond after power-up. I2C error: %d\n", ret);
        return -1;
    }

    LOG_INF("Device successfully transitioned to SOFT_PD.\n");

    return 0;
}

/* Enter Deep Power Down Mode */
void lis2dux12_sleep(stmdev_ctx_t *ctx) {
    int ret = lis2dux12_enter_deep_power_down(ctx, 1);  // Enable deep power down
    if (ret == 0) {
        LOG_INF("LIS2DUX12 entered deep power down mode successfully.\n");
    } else {
        LOG_INF("Error: Failed to enter deep power down mode.\n");
    }
}

/* Exit Deep Power Down Mode */
void lis2dux12_wakeup(stmdev_ctx_t *ctx) {
    int ret;

    LOG_INF("Exiting deep power down mode...\n");

    /* Perform the I²C-specific power-up sequence */
    ret = lis2dux12_i2c_power_up_sequence(ctx);
    if (ret != 0) {
        LOG_INF("Error: Failed to exit deep power down mode.\n");
        return;
    }

    /* Verify Communication */
    uint8_t who_am_i = 0;
    ret = lis2dux12_device_id_get(ctx, &who_am_i);
    if (ret == 0 && who_am_i == LIS2DUX12_ID) {
        LOG_INF("LIS2DUX12 exited deep power down mode successfully (WHO_AM_I: 0x%X).\n", who_am_i);
    } else {
        LOG_INF("Error: Failed to communicate with LIS2DUX12 after wake-up.\n");
    }

    /* Reinitialize Sensor Configuration */
    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,       // Set desired ODR
        .fs = LIS2DUX12_4g,              // Full scale
        .bw = LIS2DUX12_ODR_div_4,       // Bandwidth
    };
    ret = lis2dux12_mode_set(ctx, &mode);
    if (ret != 0) {
        LOG_INF("Error: Failed to reinitialize LIS2DUX12 configuration.\n");
    } else {
        LOG_INF("LIS2DUX12 reinitialized successfully.\n");
    }

    k_sleep(K_MSEC(10));
}


/* Motion Detection Setup */
static int lis2dux12_configure_motion_detection(stmdev_ctx_t *ctx) {
    uint8_t who_am_i = 0;

    // Verify device ID
    if (lis2dux12_device_id_get(ctx, &who_am_i) != 0 || who_am_i != LIS2DUX12_ID) {
        LOG_INF("Device not recognized\n");
        return -ENODEV;
    }
    
    // Enable internal pullup on INT1 pin
    uint8_t pin_ctrl_reg = 0x1A;
    if (lis2dux12_write_reg(ctx, LIS2DUX12_PIN_CTRL, &pin_ctrl_reg, 1) < 0) {
        LOG_ERR("Failed to configure PIN_CTRL");
        return -EIO;
    }

    // Enable Interrupts
    uint8_t interrupt_cfg_reg = 0x27;
    if (lis2dux12_write_reg(ctx, LIS2DUX12_INTERRUPT_CFG, &interrupt_cfg_reg, 1) < 0) {
        LOG_ERR("Failed to configure INTERRUPT_CFG");
        return -EIO;
    }

    // Map wake-up interrupt to INT1
    uint8_t md1_cfg_reg = 0x20;  // Map wake-up to INT1
    if (lis2dux12_write_reg(ctx, LIS2DUX12_MD1_CFG, &md1_cfg_reg, 1) < 0) {
        LOG_ERR("Failed to configure MD1_CFG");
        return -EIO;
    }

    // Calculate and set the wake-up threshold
    uint16_t threshold = motion_threshold_mg / 0.122; // Convert mg to LSB (±4 g scale, 0.122 mg/LSB)
    uint8_t threshold_buf = (uint8_t)(threshold & 0x3F); // WAKE_UP_THS is 6 bits wide
    threshold_buf |= (1 << 6); // Set bit 6 (SLEEP_ON) to 1. This is for activity/inactivity detection
    if (lis2dux12_write_reg(ctx, LIS2DUX12_WAKE_UP_THS, &threshold_buf, 1) < 0) {
        LOG_ERR("Failed to set WAKE_UP_THS");
        return -EIO;
    }

    // Set wake-up duration (debounce period)
    uint8_t duration = 0x00;  // Example debounce duration
    if (lis2dux12_write_reg(ctx, LIS2DUX12_WAKE_UP_DUR, &duration, 1) < 0) {
        LOG_ERR("Failed to set WAKE_UP_DUR");
        return -EIO;
    }

    // Configure CTRL1 to enable wakeup detection on all axis
    uint8_t ctrl1_reg = 0x57;
    if (lis2dux12_write_reg(ctx, LIS2DUX12_CTRL1, &ctrl1_reg, 1) < 0) {
        LOG_ERR("Failed to configure CTRL1");
        return -EIO;
    }

    // Configure CTRL4 for activity/inactivity detection
    uint8_t ctrl4_reg = 0x80;
    if (lis2dux12_write_reg(&lis_ctx, LIS2DUX12_CTRL4, &ctrl4_reg, 1) < 0) {
        LOG_ERR("Failed to configure CTRL4");
        return -EIO;
    }

    LOG_INF("LIS2DUX12 interrupt1 initialized with threshold: %d mg", motion_threshold_mg);

    return 0;
}

/* Interrupt Callback Function */
static void motion_detected_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_INF("Motion detected! Interrupt triggered...\n");
    k_sleep(K_MSEC(10));  // Allow RTT to process logs

    motion_detected = true;
}

/* Initialize GPIO Interrupt */
void lis2dux12_init_interrupt(void) {
    int ret;

    /* Configure GPIO Pin */
    if (gpio_pin_configure(gpio_dev, INT1_PIN, GPIO_INPUT | GPIO_ACTIVE_LOW) != 0) {
        LOG_INF("Error: Failed to configure GPIO pin %d.\n", INT1_PIN);
        return;
    }

    /* Configure GPIO Interrupt */
    if (gpio_pin_interrupt_configure(gpio_dev, INT1_PIN, GPIO_INT_EDGE_FALLING) != 0) {
        LOG_INF("Error: Failed to configure GPIO interrupt.\n");
        return;
    }

    gpio_init_callback(&motion_cb_data, motion_detected_callback, BIT(INT1_PIN));
    ret = gpio_add_callback(gpio_dev, &motion_cb_data);
    if (ret < 0) {
        LOG_INF("Failed to add the callback err code %d",ret);
        return;
    }

    LOG_INF("Interrupt initialized on pin %d.\n", INT1_PIN);
}



int main(void) {

    /* Initialize Sensor */
    lis2dux12_init();

    lis2dux12_init_with_retries();

    /* Perform Self-Test */
    perform_self_test();

    /* Enter deep power down mode */;
    lis2dux12_sleep(&lis_ctx);
    /* Wait for 5 seconds */
    k_sleep(K_SECONDS(5));
    /* Exit deep power down mode */
    lis2dux12_wakeup(&lis_ctx);

    /* Configure motion detection */
    lis2dux12_configure_motion_detection(&lis_ctx);

    /* Initialize Interrupt */
    lis2dux12_init_interrupt();

    uint8_t angle_mode = 7;         // Example mode, can be changed as per requirement
    uint8_t avg_accel_mode = 7;     // Example mode, can be changed as per requirement

    while (1) {

        read_lis2dux12_angle(angle_mode);           // Call the function to read angle data
        read_lis2dux12_avg_accel(avg_accel_mode);   // Call the function to read average acceleration data
        
        read_switch_monitoring();

        // read_vibration_monitoring();
        
        k_msleep(2000);
    }

    return 0;
}

