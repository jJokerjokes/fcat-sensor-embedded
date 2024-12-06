#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"

// Use the actual GPIO pin number instead of the ADC channel enum.
// ADC1_CHANNEL_6 corresponds to GPIO34 on ESP32.
#define MOISTURE_PULSE_GPIO GPIO_NUM_34  
#define POWER_GPIO GPIO_NUM_26

// Delay for counting pulses (in milliseconds)
static const int16_t SOIL_PULSE_COUNT_DELAY = 500;

static const char *TAG = "SOIL_MOISTURE";

// Configure the PCNT unit for counting pulses from the moisture sensor
static void configure_pcnt() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = MOISTURE_PULSE_GPIO, // GPIO for pulse input
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,    // No control GPIO
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        // Count pulses on the rising edge. Adjust if sensor's signal differs.
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        // Set arbitrary large limit values; adjust if needed
        .counter_h_lim = 10000,
        .counter_l_lim = 0
    };

    // Initialize the PCNT unit with the given configuration
    pcnt_unit_config(&pcnt_config);

    // Enable the input filter to debounce noise (1000 µs)
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);

    // Initialize and start the PCNT counter
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

// Read the moisture level by counting pulses over a defined interval
static int read_moisture() {
    int16_t moisture_count = 0;

    // Clear the counter and resume counting
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // Wait for the specified delay to accumulate pulses
    vTaskDelay(pdMS_TO_TICKS(SOIL_PULSE_COUNT_DELAY));

    // Retrieve the counted pulses
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);

    // Pause the counter after reading
    pcnt_counter_pause(PCNT_UNIT_0);

    return abs(moisture_count);
}

// Enable sensor power supply
static void enable_sensor_power() {
    gpio_set_level(POWER_GPIO, 1);
}

// Disable sensor power supply
static void disable_sensor_power() {
    gpio_set_level(POWER_GPIO, 0);
}

void app_main() {
    // Configure the PCNT unit for the moisture sensor
    configure_pcnt();

    // Set the power pin as output and initialize it
    gpio_set_direction(POWER_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Power on the sensor
        enable_sensor_power();

        // Read the moisture level (as pulse count)
        int moisture_value = read_moisture();
        ESP_LOGI(TAG, "Pulse Count: %d", moisture_value);

        // Power off the sensor to save energy
        disable_sensor_power();

        // Delay before the next reading
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds
    }
}
