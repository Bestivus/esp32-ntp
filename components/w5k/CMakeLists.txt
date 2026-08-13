idf_component_register(
    SRCS "w5k_udp_wrapper.c" "w5k_tcp_wrapper.c"
    INCLUDE_DIRS "."
    REQUIRES w5500_eth
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_SOURCE_DIR}/ioLibrary_Driver/Ethernet"
    "${CMAKE_SOURCE_DIR}/ioLibrary_Driver/Ethernet/W5500"
)

# Matching rename to w5500_eth's socket.c/dhcp.c fix -- these wrapper files
# call WIZnet's raw close(sn) directly (for W5500 SOCKET handles), which must
# resolve to the SAME renamed symbol as socket.c's definition, not the real
# POSIX close() that other components (e.g. ntp_stats) still use normally.
set_source_files_properties(
    "w5k_tcp_wrapper.c"
    "w5k_udp_wrapper.c"
    PROPERTIES COMPILE_DEFINITIONS "close=w5k_wiz_close"
)
