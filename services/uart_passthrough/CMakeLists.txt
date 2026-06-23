add_library(service_uart_passthrough STATIC
    uart_passthrough.c
)

target_include_directories(service_uart_passthrough
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(service_uart_passthrough
    PUBLIC
        hal_rp2040
        service_swd_bus_lock
)
