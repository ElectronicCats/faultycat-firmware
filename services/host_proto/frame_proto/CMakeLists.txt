add_library(host_proto_frame STATIC
    frame_proto.c
)
target_include_directories(host_proto_frame
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
