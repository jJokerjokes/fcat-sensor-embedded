#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/pcnt.h"
#include "driver/adc.h"

#include "ClosedCube_HDC1080.h"

// -------------------- GPIO 与硬件定义 --------------------

// 土壤湿度脉冲输入：GPIO34（输入专用引脚）
#define MOISTURE_PULSE_GPIO GPIO_NUM_34

// 传感器电源控制：GPIO26
#define POWER_GPIO GPIO_NUM_26

// I2C 引脚定义 (供 HDC1080 使用)
#define I2C_MASTER_SCL_IO  GPIO_NUM_22
#define I2C_MASTER_SDA_IO  GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_PORT           I2C_NUM_0

// 等待传感器脉冲的时间 (毫秒)
static const int16_t SOIL_PULSE_COUNT_DELAY = 500;

// 日志输出标签
static const char* TAG = "SENSOR_READINGS";

// HDC1080 传感器实例（温湿度）
static ClosedCube_HDC1080 hdc1080;

// -------------------- PCNT 相关函数 --------------------

/**
 * @brief 配置 PCNT (Pulse Counter)，用于测量从传感器输出的脉冲数量
 */
static void configure_pcnt()
{
    pcnt_config_t pcnt_config;
    memset(&pcnt_config, 0, sizeof(pcnt_config));

    // 脉冲输入引脚
    pcnt_config.pulse_gpio_num = MOISTURE_PULSE_GPIO;
    // 不使用控制引脚
    pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    // 使用 PCNT_UNIT_0, channel_0
    pcnt_config.unit    = PCNT_UNIT_0;
    pcnt_config.channel = PCNT_CHANNEL_0;

    // 在上升沿计数，下降沿不计数
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;

    // 控制模式：KEEP 即不改变计数方式
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

    // 计数上限/下限
    pcnt_config.counter_h_lim = 10000;
    pcnt_config.counter_l_lim = 0;

    // 初始化 PCNT
    pcnt_unit_config(&pcnt_config);

    // 设置滤波器（单位：µs），此处为 1000µs (1ms)
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);

    // 暂停、清零并启动计数器
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

/**
 * @brief 读取脉冲计数，用于表示土壤湿度
 * @return 返回脉冲计数值
 */
static int read_moisture()
{
    int16_t moisture_count = 0;

    // 每次读取前都要清零并启动
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // 等待一段时间，让脉冲得以累计
    vTaskDelay(pdMS_TO_TICKS(SOIL_PULSE_COUNT_DELAY));

    // 读取当前计数器数值
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);

    // 暂停计数器
    pcnt_counter_pause(PCNT_UNIT_0);

    // 返回绝对值
    return std::abs(moisture_count);
}

// -------------------- 电源控制 --------------------

/**
 * @brief 打开传感器电源
 */
static void enable_sensor_power()
{
    gpio_set_level(POWER_GPIO, 1);
}

/**
 * @brief 关闭传感器电源
 */
static void disable_sensor_power()
{
    gpio_set_level(POWER_GPIO, 0);
}

// -------------------- I2C 初始化 --------------------

/**
 * @brief 初始化 I2C，用于与 HDC1080 通信 (ESP-IDF 风格)
 */
static void i2c_master_init()
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));

    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

// -------------------- GPIO 初始化 --------------------

/**
 * @brief 配置 GPIO 用于电源控制，以及土壤湿度脉冲输入
 */
static void gpio_init_all()
{
    // 配置电源引脚为输出
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 配置土壤湿度脉冲引脚为输入（若有需要上拉下拉可在此更改）
    gpio_config_t moisture_conf = {
        .pin_bit_mask = (1ULL << MOISTURE_PULSE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&moisture_conf);

    // 初始化后，先关闭电源
    disable_sensor_power();
}

// -------------------- 主函数入口 (ESP-IDF) --------------------

extern "C" void app_main()
{
    // 初始化 GPIO
    gpio_init_all();

    // 初始化 I2C 并配置 HDC1080
    i2c_master_init();
    hdc1080.begin(0x40);

    // 初始化 PCNT，用于脉冲计数
    configure_pcnt();

    while (true)
    {
        // 1. 打开传感器电源
        enable_sensor_power();
        // 稍等一下，给传感器上电时间（50ms）
        vTaskDelay(pdMS_TO_TICKS(50));

        // 2. 读取土壤湿度（脉冲计数）
        int moisture_value = read_moisture();

        // 3. 读取温湿度
        float temperatureC = hdc1080.readTemperature();
        float humidity     = hdc1080.readHumidity();
        float temperatureF = temperatureC * 1.8f + 32.0f;

        // 4. 打印测量结果
        ESP_LOGI(TAG, "Moisture Pulse Count: %d", moisture_value);
        ESP_LOGI(TAG, "Temperature: %.2f °C (%.2f °F)", temperatureC, temperatureF);
        ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);

        // 5. 关闭传感器电源（节省能耗）
        disable_sensor_power();

        // 6. 等待 5s 再进行下一轮测量
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
