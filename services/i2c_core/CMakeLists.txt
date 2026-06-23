add_library(service_i2c STATIC
    i2c_core.c
    i2c_la.c
)

target_include_directories(service_i2c
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(service_i2c
    PUBLIC
        hal_rp2040
        board_v2_header
)
