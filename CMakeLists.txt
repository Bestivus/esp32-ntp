cmake_minimum_required(VERSION 3.16)
# SPDX-License-Identifier: Unlicense
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_ntp)
idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)