# rpi8080
An rp2040 based intel 8080 emulator with debugger, physical &amp; virtual IN/OUT port and interrupt.

### Note: The project code is fairly complete and usable, but the documentation is not ready yet.

In this project, 8080's emulation core is based on (superzazu 8080)[https://github.com/superzazu/8080]'s work.

In order to use the project, first build it using `pico-sdk` and `cmake` and flash it onto your favorite
rp2040 development board. You might want to disable the default input/output pins or change their configuration.

Next, write your code in (asm80)[www.asm80.com]. After reset `SP` will be at `0xFFF0` and `PC` starts at `0x0100` so adjust 
your `ORG` accordingly. After verifying your assembly program, use "Download BIN" option and save the binary. After this run
`python3 programmer.py [rp2040 serial port] output.bin` to program your virtual 8080. You have the full 64kB of 8080's address space
to your code as R/W memory. Note that your binary will always be saved in rpi's SRAM, so you have to re-program every time you
reset your board. By default, the CPU will run at `1us` per `T-state` but you can make this slower.

After this, open a serial monitor (like `screen` or `putty`) and write anything to start the CPU. If you write `help` it will
show the supported debug commands.
