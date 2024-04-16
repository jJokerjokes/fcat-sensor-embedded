#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>


timeval start;
void setup() {
    Serial.begin(115200);
    gettimeofday(&start, NULL);
    Serial.println("Wakeup: " + String(start.tv_sec) + "." + String(start.tv_usec));
}

void sleep() {
    // gettimeofday(&tv_now, NULL); // get time of day
    // Calculates time it takes between startup and now
    uint64_t sleepTime = 10 * 60 * 1000000;//((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec;

    // Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
    Serial.println("Sleeping for 10 minutes");
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
    esp_deep_sleep_start();
}

void loop() {
    sleep();
}