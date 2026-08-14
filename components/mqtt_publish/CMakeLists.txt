idf_component_register(
    SRCS "mqtt_publish.cpp" "mqtt_wire.c"
    INCLUDE_DIRS "."
    REQUIRES gps ntp_server w5500_eth w5k esp_timer
)
