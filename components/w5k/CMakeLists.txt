idf_component_register(
    SRCS "w5k_udp_wrapper.c" "w5k_tcp_wrapper.c"
    INCLUDE_DIRS "."
    REQUIRES w5500_eth
)
