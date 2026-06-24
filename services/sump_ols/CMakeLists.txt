add_library(service_sump_ols STATIC
    sump_ols.c
)

target_include_directories(service_sump_ols
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(service_sump_ols
    PUBLIC
        hal_rp2040
        service_i2c
)
