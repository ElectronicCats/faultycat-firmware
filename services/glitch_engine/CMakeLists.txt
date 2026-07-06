# Header-only compiler shared by emfi/emfi_pio.c and crowbar/crowbar_pio.c
# (pio_glitch_prog.h). INTERFACE-only so it never becomes its own linked
# .o — both engines' static libraries can embed the same static-inline
# functions without a duplicate-symbol clash when they link into one
# firmware image.
add_library(glitch_pio_prog_header INTERFACE)
target_include_directories(glitch_pio_prog_header
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(emfi)
add_subdirectory(crowbar)
