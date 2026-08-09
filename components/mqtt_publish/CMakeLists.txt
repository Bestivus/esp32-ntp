set(MQTT_SRCS
    "../../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.c"
    "../../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.c"
    "../../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.c"
)
idf_component_register(
    SRCS "mqtt_publish.cpp" ${MQTT_SRCS}
    INCLUDE_DIRS "." "../../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src"
    REQUIRES gps ntp_server w5500_eth w5k esp_timer
)

# mqtt_publish.cpp includes w5k_tcp_wrapper.h, which includes WIZnet's own
# (unqualified) "socket.h". Requiring ntp_server also pulls in lwip's public
# include dirs transitively, and in this component's particular
# requires-closure order those land ahead of w5500_eth's Ethernet dir, so the
# bare "socket.h" resolves to lwip's shim instead of WIZnet's -- redefining
# WIZnet's SOCK_STREAM/SOCK_DGRAM aliases (see wizchip_conf.h) with lwip's
# real POSIX values and failing to compile. Same fix w5k's own CMakeLists.txt
# already applies for its sources: pin the real directory explicitly so it
# wins the search regardless of REQUIRES ordering elsewhere.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_SOURCE_DIR}/ioLibrary_Driver/Ethernet"
)

# Only the client-side wire-format codec and the non-blocking packet reader
# are vendored here (MQTTPacket.c/MQTTConnectClient.c/MQTTSerializePublish.c).
# MQTTClient.c and mqtt_interface.c (the vendored library's synchronous,
# Network/Timer-based orchestration) are deliberately NOT used -- see the
# comment at the top of mqtt_publish.h for why: this component drives the
# same wire format with its own tick-by-tick, always-non-blocking state
# machine instead, so it can share ntp_task without ever risking a stall on
# the NTP reply path.
