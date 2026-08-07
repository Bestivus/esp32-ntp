set(W5500_SRCS
    "../../ioLibrary_Driver/Ethernet/W5500/w5500.c"
    "../../ioLibrary_Driver/Ethernet/wizchip_conf.c"
    "../../ioLibrary_Driver/Ethernet/socket.c"
    "../../ioLibrary_Driver/Internet/DHCP/dhcp.c"
)
idf_component_register(
    SRCS "w5500_eth.cpp" ${W5500_SRCS}
    INCLUDE_DIRS "." "../../ioLibrary_Driver/Ethernet" "../../ioLibrary_Driver/Internet/DHCP"
    REQUIRES driver esp_driver_gpio esp_netif config efuse esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC _WIZCHIP_=5500)
# Disable specific warnings for external library code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-misleading-indentation)

# WIZnet's socket.c defines its own close(sn) (for closing W5500 SOCKET
# handles, unrelated to POSIX file descriptors) and dhcp.c calls it -- both
# collide at link time with the real close() in ESP-IDF's C library, which
# other components (e.g. ntp_stats) use for real network sockets. Renaming
# ONLY within these two files -- their own internal close()/close(sn) calls
# consistently target the same renamed symbol as the (also renamed)
# definition, so this cannot affect anything outside these two files.
set_source_files_properties(
    "../../ioLibrary_Driver/Ethernet/socket.c"
    "../../ioLibrary_Driver/Internet/DHCP/dhcp.c"
    PROPERTIES COMPILE_DEFINITIONS "close=w5k_wiz_close"
)
