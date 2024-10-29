#include "driver/adc.h"        // Include ADC driver
#include "esp_log.h"           // For logging output
#include "driver/gpio.h"       // Include GPIO control
#include "ClosedCube_HDC1080.h"// Include HDC1080 humidity sensor driver
#include "freertos/FreeRTOS.h" // FreeRTOS header for tasks and delays
#include "freertos/task.h"     // FreeRTOS task control

// Define pins
#define HUMIDITY_POWER_PIN GPIO_NUM_26  // Sensor power pin (GPIO 26)
#define MOISTURE_PIN ADC1_CHANNEL_6     // Soil moisture sensor pin (GPIO 34)

ClosedCube_HDC1080 hdc1080;  // Instantiate HDC1080 sensor

// Set up the humidity sensor pin and power on the sensor
void setup_humidity_pin() {
    gpio_set_direction(HUMIDITY_POWER_PIN, GPIO_MODE_OUTPUT);  // Set sensor power pin as output
    gpio_set_level(HUMIDITY_POWER_PIN, 1);  // Turn on power
    hdc1080.begin(0x40);  // Initialize humidity sensor (I2C address: 0x40)
}

// Set up the soil moisture sensor pin (GPIO34, ADC1 Channel 6)
void setup_moisture_pin() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC resolution to 12 bits
    adc1_config_channel_atten(MOISTURE_PIN, ADC_ATTEN_DB_11);  // Set attenuation to adapt to 0-3.3V
}

// Read data from the humidity sensor
float read_humidity() {
    return hdc1080.readHumidity();  // Read humidity percentage
}

// Read the ADC value from the soil moisture sensor
int read_moisture() {
    return adc1_get_raw(MOISTURE_PIN);  // Get ADC value from soil moisture sensor
}

void app_main() {
    // Initialize humidity and soil moisture sensor pins
    setup_humidity_pin();
    setup_moisture_pin();

    // Initialize logging system
    esp_log_level_set("SENSOR", ESP_LOG_INFO);

    while (true) {
        // Power on the sensor
        gpio_set_level(HUMIDITY_POWER_PIN, 1);  
        
        // Read humidity sensor data
        float humidity = read_humidity();
        // Read soil moisture sensor data
        int moisture = read_moisture();
        
        // Print sensor data
        ESP_LOGI("SENSOR", "Humidity: %.2f%%", humidity);  // Print humidity value
        ESP_LOGI("SENSOR", "Soil Moisture ADC Value: %d", moisture);  // Print soil moisture value

        // Power off the sensor to save energy
        gpio_set_level(HUMIDITY_POWER_PIN, 0);

        // Read every 5 seconds
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
