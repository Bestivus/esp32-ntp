idf_component_register(
    SRCS "w5500_eth.cpp" "w5500_drv.c" "w5500_dhcp.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_netif config efuse esp_timer
)
