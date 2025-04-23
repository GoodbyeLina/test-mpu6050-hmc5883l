idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." "../components/BUS"
                    REQUIRES  hmc5883l my_mpu6050 BUS sc7a20h)