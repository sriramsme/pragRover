idf_component_register(SRCS "main.c" "hardware.c" "../core/control.c" "../core/wire.c"
    INCLUDE_DIRS "." "../core"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_timer esp_wifi
             esp_event esp_netif nvs_flash json lwip)
target_compile_features(${COMPONENT_LIB} PUBLIC c_std_11)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
