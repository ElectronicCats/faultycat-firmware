# Faulty Cat

> ## Firmware v3 — rewritten from scratch
>
> This repository ships **firmware v3** for the existing FaultyCat
> v2.x hardware. It is a from-scratch rewrite of the original v2.x
> firmware, not an evolution of it — same board, new stack.

### Building the firmware

The fastest way to build the firmware from source is the official
**Raspberry Pi Pico** extension for VS Code — it installs the
toolchain (cmake, ninja, arm-none-eabi-gcc) and the pico-sdk for
you, and runs the cmake configure/build steps from the editor.

1. Install the
   [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
   extension from the VS Code Marketplace.
2. Clone this repository **with submodules**:
   `git clone --recursive <repo-url>`
   (or, after a plain clone, run
   `git submodule update --init --recursive`).
3. Open the cloned folder in VS Code, then run
   **Raspberry Pi Pico: Import Project** from the command palette
   (Ctrl+Shift+P) and point it at this folder.
4. Hit **Compile** in the status bar. The resulting `.uf2` lands
   under `build/.../apps/faultycat_fw/faultycat.uf2`.

Flash that `.uf2` using one of the paths in **Programming the
Faulty Cat** above.

## License

This project FaultyCat is adapted from [ChipSHOUTER PicoEMP](https://github.com/newaetech/chipshouter-picoemp) by [Colin O'Flynn](https://github.com/colinoflynn) is licensed under CC BY-SA 3.0, "FaultyCat" contains modifications such as: porting the project to Kicad, modifying BOM and dimensions is licensed under CC BY-SA 3.0 by ElectronicCats.

Electronic Cats invests time and resources in providing this open-source design. Please support Electronic Cats and open-source hardware by purchasing products from Electronic Cats!
