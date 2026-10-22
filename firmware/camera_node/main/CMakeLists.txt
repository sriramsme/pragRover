idf_component_register(SRCS "camera_node.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi esp_event esp_netif nvs_flash esp_timer esp_psram esp_hw_support)
target_compile_features(${COMPONENT_LIB} PRIVATE c_std_11)
