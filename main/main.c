#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "ili9340.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#include "driver/gpio.h"
#include "hmc5883l.h"
#include "my_mpu6050.h"
#include "my_i2c.h"
#include "sc7a20h.h"

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

TFT_t dev;

#define TAG "HMC5883L_APP"

TickType_t FillTest(TFT_t * dev, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdFillScreen(dev, RED);
	lcdDrawFinish(dev);
	vTaskDelay(50);
	lcdFillScreen(dev, GREEN);
	lcdDrawFinish(dev);
	vTaskDelay(50);
	lcdFillScreen(dev, BLUE);
	lcdDrawFinish(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ColorBarTest(TFT_t * dev, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	if (width < height) {
		uint16_t y1,y2;
		y1 = height/3;
		y2 = (height/3)*2;
		lcdDrawFillRect(dev, 0, 0, width-1, y1-1, RED);
		vTaskDelay(1);
		lcdDrawFillRect(dev, 0, y1-1, width-1, y2-1, GREEN);
		vTaskDelay(1);
		lcdDrawFillRect(dev, 0, y2-1, width-1, height-1, BLUE);
		lcdDrawFinish(dev);
	} else {
		uint16_t x1,x2;
		x1 = width/3;
		x2 = (width/3)*2;
		lcdDrawFillRect(dev, 0, 0, x1-1, height-1, RED);
		vTaskDelay(1);
		lcdDrawFillRect(dev, x1-1, 0, x2-1, height-1, GREEN);
		vTaskDelay(1);
		lcdDrawFillRect(dev, x2-1, 0, width-1, height-1, BLUE);
		lcdDrawFinish(dev);
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ArrowTest(TFT_t * dev, FontxFile *fx, uint16_t model, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	ESP_LOGD(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	int	stlen;
	uint8_t ascii[24];
	uint16_t color;

	lcdFillScreen(dev, BLACK);

	if (model == 0x9225) strcpy((char *)ascii, "ILI9225");
	if (model == 0x9340) strcpy((char *)ascii, "ILI9340");
	if (model == 0x9341) strcpy((char *)ascii, "ILI9341");
	if (model == 0x7735) strcpy((char *)ascii, "ST7735");
	if (model == 0x7789) strcpy((char *)ascii, "ST7789");
	if (model == 0x7796) strcpy((char *)ascii, "ST7796");
	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	lcdSetFontDirection(dev, DIRECTION0);
	//lcdFillScreen(dev, WHITE);
	color = RED;
	lcdDrawFillArrow(dev, 10, 10, 0, 0, 5, color);
	strcpy((char *)ascii, "0,0");
	lcdDrawString(dev, fx, 0, 30, ascii, color);

	color = GREEN;
	lcdDrawFillArrow(dev, width-11, 10, width-1, 0, 5, color);
	//strcpy((char *)ascii, "79,0");
	sprintf((char *)ascii, "%d,0",width-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, fx, xpos, 30, ascii, color);

	color = GRAY;
	lcdDrawFillArrow(dev, 10, height-11, 0, height-1, 5, color);
	//strcpy((char *)ascii, "0,159");
	sprintf((char *)ascii, "0,%d",height-1);
	ypos = (height-11) - (fontHeight) + 5;
	lcdDrawString(dev, fx, 0, ypos, ascii, color);

	color = CYAN;
	lcdDrawFillArrow(dev, width-11, height-11, width-1, height-1, 5, color);
	//strcpy((char *)ascii, "79,159");
	sprintf((char *)ascii, "%d,%d",width-1, height-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	lcdDrawFinish(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t Sensor_Test(TFT_t * dev, FontxFile *fx, uint16_t model, int width, int height, float Pitch, float Roll, float Yaw) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	ESP_LOGD(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	int	stlen;
	uint8_t ascii_1[24];
	uint8_t ascii_2[24];
	uint8_t ascii_3[24];

	uint16_t color;

	lcdFillScreen(dev, BLACK);

	// 俯仰角（Pitch）
	// strcpy((char *)ascii_1, "Pitch: 0");
	sprintf((char *)ascii_1, "Pitch: %.2f", Pitch);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1 + 36;
		ypos = (height - (strlen((char *)ascii_1) * fontWidth)) / 2 - 36;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii_1) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos, ascii_1, color);

	// 横滚角（Roll）
	// strcpy((char *)ascii_2, "Roll: 0");
	sprintf((char *)ascii_2, "Roll: %.2f", Roll);

	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1 + 12;
		ypos = (height - (strlen((char *)ascii_2) * fontWidth)) / 2 - 36;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii_2) * fontWidth)) / 2;
		// lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos, ascii_2, color);

	// 航向角（Yaw）
	// strcpy((char *)ascii_3, "Yaw: 0");
	sprintf((char *)ascii_3, "Yaw: %.2f", Yaw);

	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1 - 12;
		ypos = (height - (strlen((char *)ascii_3) * fontWidth)) / 2 - 36;
		// lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii_3) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos, ascii_3, color);

	lcdDrawFinish(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}


static void listSPIFFS(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

esp_err_t mountSPIFFS(char * path, char * label, int max_files) {
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = max_files,
		.format_if_mount_failed = true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return ret;
	}

#if 0
	ESP_LOGI(TAG, "Performing SPIFFS_check().");
	ret = esp_spiffs_check(conf.partition_label);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
		return ret;
	} else {
			ESP_LOGI(TAG, "SPIFFS_check() successful");
	}
#endif

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}


void app_main(void)
{
    // 初始化I2C总线(使用默认配置)
    i2c_bus_init();

    // 初始化HMC5883L(使用默认配置)
    if (hmc5883l_init(NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return;
    }
    ESP_LOGI(TAG, "HMC5883L initialized successfully");

    hmc5883l_data_t hcm5883l_data;
    
    // 初始化mpu6050(使用默认配置)
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    my_mpu6050_init();

    ret = mpu6050_get_deviceid(my_mpu6050_get_handle(), &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

	float gyro_offsets[3];
    calibrate_gyro(my_mpu6050_get_handle(), gyro_offsets);

    // 初始化 sc7a20h(使用默认配置)
    for (int i = 0; i < 3; i++) {
        esp_err_t err = sc7a20h_init();
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "Init attempt %d failed", i+1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    int16_t x, y, z;

    // 初始化1.8寸屏幕（ST7735）
    // Initialize NVS
	// NVS saves the touch position calibration.
	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	ESP_LOGI(TAG, "Initializing SPIFFS");
	// Maximum files that could be open at the same time is 10.
	ESP_ERROR_CHECK(mountSPIFFS("/fonts", "storage0", 10));
	listSPIFFS("/fonts/");

	// Image file borrowed from here
	// https://www.flaticon.com/packs/social-media-343
	// Maximum files that could be open at the same time is 1.
	ESP_ERROR_CHECK(mountSPIFFS("/icons", "storage1", 1));
	listSPIFFS("/icons/");

	// Maximum files that could be open at the same time is 1.
	ESP_ERROR_CHECK(mountSPIFFS("/images", "storage2", 1));
	listSPIFFS("/images/");

    FontxFile fx16G[2];
	FontxFile fx24G[2];
	FontxFile fx32G[2];
	InitFontx(fx16G,"/fonts/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/fonts/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/fonts/ILGH32XB.FNT",""); // 16x32Dot Gothic

	FontxFile fx32L[2];
	InitFontx(fx32L,"/fonts/LATIN32B.FNT",""); // 16x32Dot Latinc

	FontxFile fx16M[2];
	FontxFile fx24M[2];
	FontxFile fx32M[2];
	InitFontx(fx16M,"/fonts/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/fonts/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/fonts/ILMH32XB.FNT",""); // 16x32Dot Mincyo

	ESP_LOGI(TAG, "Disable Touch Contoller");
	int XPT_MISO_GPIO = -1;
	int XPT_CS_GPIO = -1;
	int XPT_IRQ_GPIO = -1;
	int XPT_SCLK_GPIO = -1;
	int XPT_MOSI_GPIO = -1;
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_TFT_CS_GPIO, CONFIG_DC_GPIO, 
		CONFIG_RESET_GPIO, CONFIG_BL_GPIO, XPT_MISO_GPIO, XPT_CS_GPIO, XPT_IRQ_GPIO, XPT_SCLK_GPIO, XPT_MOSI_GPIO);
    
    #if CONFIG_ST7735
        uint16_t model = 0x7735;
    #endif
        lcdInit(&dev, model, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
    
    
    // 循环读取任务
    while (1) {
    // 读取HMC5883L数据
/*         if (hmc5883l_read(&hcm5883l_data) == ESP_OK) {
            ESP_LOGI(TAG, "X:%.2fG Y:%.2fG Z:%.2fG Heading:%.1f°",
                    hcm5883l_data.x, hcm5883l_data.y, hcm5883l_data.z, hcm5883l_data.heading);
        } else {
            ESP_LOGE(TAG, "Failed to read hcm5883l_data");
        }
    
 */	// 读取mpu6050数据
        ret = mpu6050_get_acce(my_mpu6050_get_handle(), &acce);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
    
        ret = mpu6050_get_gyro(my_mpu6050_get_handle(), &gyro);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
		
		gyro.gyro_x -= gyro_offsets[0];
        gyro.gyro_y -= gyro_offsets[1];
        gyro.gyro_z -= gyro_offsets[2];

        // 姿态解算
        static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        static uint32_t last_time = 0;
        const float alpha = 0.96f;
        
        // 计算时间差（单位：秒）
        float dt = (xTaskGetTickCount() - last_time) * portTICK_PERIOD_MS / 1000.0f;
        last_time = xTaskGetTickCount();

        // 加速度计姿态
        float acc_roll = atan2f(acce.acce_y, acce.acce_z) * 57.2958f;
        float acc_pitch = atan2f(-acce.acce_x, sqrtf(acce.acce_y*acce.acce_y + acce.acce_z*acce.acce_z)) * 57.2958f;

        // 陀螺仪积分
        roll = alpha * (roll + gyro.gyro_x * dt) + (1 - alpha) * acc_roll;
        pitch = alpha * (pitch + gyro.gyro_y * dt) + (1 - alpha) * acc_pitch;
        yaw += gyro.gyro_z * dt;

        // 限制角度范围
        if (yaw > 180.0f) yaw -= 360.0f;
        else if (yaw < -180.0f) yaw += 360.0f;

        // 完整姿态信息输出
        ESP_LOGI(TAG, "Attitude: Roll=%.2f° Pitch=%.2f° Yaw=%.2f°", roll, pitch, yaw);

    // 读取 sc7a20h 数据
/*         if (sc7a20h_read_accel(&x, &y, &z) == ESP_OK) {
            ESP_LOGI(TAG, "Accel X:%.2fg Y:%.2fg Z:%.2fg\n",
                    x * 0.004f, y * 0.004f, z * 0.004f);
        }
 */
    // TFT显示屏显示任务
		Sensor_Test(&dev, fx16G, model, CONFIG_WIDTH, CONFIG_HEIGHT, pitch, roll, yaw);
		WAIT;

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    mpu6050_delete(my_mpu6050_get_handle());
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

}
