idf_component_register(SRCS "decode_png.c" "decode_jpeg.c" "pngle.c" "main.c"
                    INCLUDE_DIRS "." "../components/BUS"
                    REQUIRES vfs spiffs nvs_flash hmc5883l my_mpu6050 BUS sc7a20h ili9340)