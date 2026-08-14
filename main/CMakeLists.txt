idf_component_register(
  SRCS "app_main.cpp"
  INCLUDE_DIRS "."
  REQUIRES config display gps ntp_server webui w5500_eth wifi_sta mqtt_publish esp_event esp_timer nvs_flash esp_netif esp_driver_uart
)


