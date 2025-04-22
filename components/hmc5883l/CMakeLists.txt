idf_component_register(SRCS "hmc5883l.c"
                    INCLUDE_DIRS "." "../BUS"
                    REQUIRES driver BUS)