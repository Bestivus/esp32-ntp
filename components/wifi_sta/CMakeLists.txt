set(srcs)
if(CONFIG_SOC_WIFI_SUPPORTED)
    list(APPEND srcs "wifi_sta.cpp")
endif()

idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "." REQUIRES esp_wifi esp_netif esp_event)
