#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"

#define MOISTURE_PULSE_GPIO ADC1_CHANNEL_6  // GPIO 34 for ADC1_CHANNEL_6
#define POWER_GPIO GPIO_NUM_26
int16_t SOIL_PULSE_COUNT_DELAY = 500 ;    // Delay for pulse count in milliseconds

// Function to configure the PCNT unit
void configure_pcnt() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = MOISTURE_PULSE_GPIO,  // GPIO for pulse input
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,    // No control pin
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,            // Count rising edges
        .neg_mode = PCNT_COUNT_DIS,           // Ignore falling edges
        .lctrl_mode = PCNT_MODE_KEEP,         // Keep the counter mode
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 10000,               // High limit (arbitrary large value)
        .counter_l_lim = 0                    // Low limit
    };

    // Initialize the PCNT unit
    pcnt_unit_config(&pcnt_config);

    // Enable counter filtering to avoid noise
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);  // 1000 us filter
    pcnt_filter_enable(PCNT_UNIT_0);

    // Initialize counter to zero
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    // Enable the counter
    pcnt_counter_resume(PCNT_UNIT_0);
}

// Function to read soil moisture
int read_moisture() {
    int16_t moisture_count = 0;

    // Clear and start counting
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // Wait for the specified delay
    vTaskDelay(pdMS_TO_TICKS(SOIL_PULSE_COUNT_DELAY));

    // Get the pulse count
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);

    // Pause the counter
    pcnt_counter_pause(PCNT_UNIT_0);

    return abs(moisture_count);  // Return absolute value of pulse count
}

// Function to enable and disable sensor power
void enable_sensor_power() {
    gpio_set_direction(POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_GPIO, 1);  // Power on
}

void disable_sensor_power() {
    gpio_set_level(POWER_GPIO, 0);  // Power off
}

// Main function
void app_main() {
    // Configure GPIO and PCNT
    configure_pcnt();
    gpio_set_direction(POWER_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Enable sensor power
        enable_sensor_power();

        // Read soil moisture
        int moisture_value = read_moisture();
        ESP_LOGI("SOIL MOISTURE", "Pulse Count: %d", moisture_value);

        // Disable sensor power to save energy
        disable_sensor_power();

        // Delay before next reading
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds
    }
}
