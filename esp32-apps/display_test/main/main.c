#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"

void app_main(void){
	SSD1306_t dev;
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);	
	ssd1306_init(&dev, 128, 64);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
  	ssd1306_display_text(&dev, 1, "IP:          ", 16, false);
	ssd1306_display_text(&dev, 2, "192.168.0.106", 16, false);
 	vTaskDelay(10000 / portTICK_PERIOD_MS);
	ssd1306_fadeout(&dev);
	esp_restart();
}