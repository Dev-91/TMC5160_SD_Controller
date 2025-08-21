idf_component_register(
    SRCS "main.c"
         "driver/button_driver.c"
         "driver/photo_interrupt.c"
         "driver/tmc5160_SD_driver.c"
    INCLUDE_DIRS "."
                 "driver"
)