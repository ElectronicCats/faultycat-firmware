add_library(service_logic_analyzer STATIC
    logic_analyzer.c
)

target_include_directories(service_logic_analyzer
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(service_logic_analyzer
    PUBLIC
        hal_rp2040
        board_v2_header
)
