cmake_minimum_required(VERSION 3.20.0)

set(BOARD nucleo_g474re)

set(CONF_FILE "boards/nucleo_g474re.conf")
list(APPEND CONF_FILE "boards/nucleo_g474re.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(se28cp_app)

target_sources(app PRIVATE src/dac.c)
#src/main.c

