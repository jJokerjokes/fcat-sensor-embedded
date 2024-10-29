#include "driver/adc.h"        // 包含 ADC 驱动
#include "esp_log.h"           // 用于日志输出
#include "driver/gpio.h"       // 包含 GPIO 控制
#include "ClosedCube_HDC1080.h"// 包含湿度传感器 HDC1080 驱动
#include "freertos/FreeRTOS.h" // FreeRTOS 头文件，用于任务和延时
#include "freertos/task.h"     // FreeRTOS 的任务控制

// 定义引脚
#define HUMIDITY_POWER_PIN GPIO_NUM_26  // 传感器电源引脚（GPIO 26）
#define MOISTURE_PIN ADC1_CHANNEL_6     // 土壤湿度传感器引脚（GPIO 34）

ClosedCube_HDC1080 hdc1080;  // 实例化 HDC1080 传感器

// 设置湿度传感器引脚，并开启传感器电源
void setup_humidity_pin() {
    gpio_set_direction(HUMIDITY_POWER_PIN, GPIO_MODE_OUTPUT);  // 设置传感器电源引脚为输出模式
    gpio_set_level(HUMIDITY_POWER_PIN, 1);  // 打开电源
    hdc1080.begin(0x40);  // 初始化湿度传感器 (I2C地址: 0x40)
}

// 设置土壤湿度传感器引脚 (GPIO34, ADC1通道6)
void setup_moisture_pin() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // 设置ADC分辨率为12位
    adc1_config_channel_atten(MOISTURE_PIN, ADC_ATTEN_DB_11);  // 设置衰减，使其适应0-3.3V电压
}

// 读取湿度传感器的数据
float read_humidity() {
    return hdc1080.readHumidity();  // 读取湿度百分比
}

// 读取土壤湿度传感器的ADC值
int read_moisture() {
    return adc1_get_raw(MOISTURE_PIN);  // 获取土壤湿度传感器的ADC值
}

void app_main() {
    // 初始化湿度和土壤湿度传感器引脚
    setup_humidity_pin();
    setup_moisture_pin();

    // 日志系统初始化
    esp_log_level_set("SENSOR", ESP_LOG_INFO);

    while (true) {
        // 打开传感器电源
        gpio_set_level(HUMIDITY_POWER_PIN, 1);  
        
        // 读取湿度传感器数据
        float humidity = read_humidity();
        // 读取土壤湿度传感器数据
        int moisture = read_moisture();
        
        // 打印传感器数据
        ESP_LOGI("SENSOR", "Humidity: %.2f%%", humidity);  // 打印湿度值
        ESP_LOGI("SENSOR", "Soil Moisture ADC Value: %d", moisture);  // 打印土壤湿度值

        // 关闭传感器电源以节约能耗
        gpio_set_level(HUMIDITY_POWER_PIN, 0);

        // 每5秒读取一次
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
