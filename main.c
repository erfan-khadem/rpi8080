#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/resets.h"

#include "i8080.h"

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint8_t u8;

typedef int64_t i64;
typedef int32_t i32;
typedef int8_t i8;

#define EMULATOR_VERSION 0x0100

#define CRLF "\r\n"
#define MEM_SIZE 65536

#define INTR_EN (1 << 8)
#define INTR_MSK (INTR_EN - 1)

// IN PORT Definitions

#define IN_PIN_BASE_PORT1 16
#define IN_PIN_MSK_PORT1 (0x7f << IN_PIN_BASE_PORT1)
#define IN_ADDR_PORT1 2

// OUT PORT Definitions

#define OUT_PIN_BASE_PORT1 0
#define OUT_PIN_BASE_PORT2 8

#define OUT_PIN_MSK_PORT1 (0xff << OUT_PIN_BASE_PORT1)
#define OUT_PIN_MSK_PORT2 (0xff << OUT_PIN_BASE_PORT2)

#define OUT_ADDR_PORT1 0
#define OUT_ADDR_PORT2 1

#define LED_PIN 25

i8080 cpu;

uint8_t memory[MEM_SIZE] __aligned(MEM_SIZE);
uint8_t out_port[256];
uint8_t in_port[256];

uint32_t intr_status = 0;
uint32_t delay_mlt = 1;

bool memory_ready = false;
bool cpu_stopped = false;

void init_gpio() {
    gpio_put_masked(OUT_PIN_MSK_PORT1, 0);
    gpio_put_masked(OUT_PIN_MSK_PORT2, 0);
    gpio_init_mask(OUT_PIN_MSK_PORT1 | OUT_PIN_MSK_PORT2 | IN_PIN_MSK_PORT1);
    gpio_set_dir_masked(OUT_PIN_MSK_PORT1 | OUT_PIN_MSK_PORT2 | IN_PIN_MSK_PORT1,
                        OUT_PIN_MSK_PORT1 | OUT_PIN_MSK_PORT2);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);
    gpio_put(LED_PIN, false);
}

u8 read_byte_mem(void* user_data, uint16_t addr) {
    return memory[addr];
}

void write_byte_mem(void* user_data, uint16_t addr, uint8_t data) {
    memory[addr] = data;
}

u8 port_in(void* user_data, uint8_t addr) {
    if (addr == IN_ADDR_PORT1) {
        uint8_t data = (gpio_get_all() & IN_PIN_MSK_PORT1) >> IN_PIN_BASE_PORT1;
        in_port[addr] = data;
        return data;
    } else {
        return in_port[addr];
    }
}

void port_out(void* user_data, uint8_t addr, uint8_t data) {
    if (addr == OUT_ADDR_PORT1) {
        gpio_put_masked(OUT_PIN_MSK_PORT1, data << OUT_PIN_BASE_PORT1);
    } else if (addr == OUT_ADDR_PORT2) {
        gpio_put_masked(OUT_PIN_MSK_PORT2, data << OUT_PIN_BASE_PORT2);
    }
    out_port[addr] = data;
}

inline void handle_intr(i8080* const c) {
    if (intr_status & INTR_EN) {
        intr_status = intr_status & INTR_MSK;
        i8080_interrupt(c, (uint8_t)(intr_status & INTR_MSK));
    }
}

void reset_cpu(i8080* const c) {
    i8080_init(c);
    c->userdata = c;
    c->read_byte = read_byte_mem;
    c->write_byte = write_byte_mem;
    c->port_in = port_in;
    c->port_out = port_out;

    c->sp = 0xfff0;
    c->pc = 0x0100;

    intr_status = 0;

    memset(out_port, 0, 256);
}

void core1_entry() {
    multicore_fifo_pop_blocking();  // Wait for memory programming

    uint32_t curr_time = 0;
    uint32_t target = 0;
    uint64_t prev_cyc = 0;

    i8080* c = &cpu;
    reset_cpu(c);

    multicore_fifo_push_blocking(1);
    sleep_ms(2);

    while (true) {
        while ((curr_time = time_us_32()) < target) {
            tight_loop_contents();
        }
        if (multicore_fifo_rvalid()) {
            multicore_fifo_pop_blocking();
            cpu_stopped = true;
            multicore_fifo_pop_blocking();
            cpu_stopped = false;
            curr_time = time_us_32();
            prev_cyc = c->cyc;
        }
        handle_intr(c);
        i8080_step(c);
        target = curr_time + delay_mlt * (c->cyc - prev_cyc);
        if (target < curr_time) {  // Time overflow
            // This happens every ~70 minutes. At most one additional cycle happens every 70 minutes.
            busy_wait_us(delay_mlt * (c->cyc - prev_cyc));  // This CPU cycle might take a little longer.
        }
        prev_cyc = c->cyc;
    }
}

int main() {
    init_gpio();

    stdio_usb_init();

START_OVER:
    intr_status = 0;
    delay_mlt = 1;
    memset(out_port, 0, 256);
    memset(in_port, 0, 256);
    cpu_stopped = false;
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);
    if (memory == NULL) {
        // Blink the LED rapidly to indicate a critical error (memory allocation failed)
        while (true) {
            gpio_put(LED_PIN, true);
            busy_wait_ms(100);
            gpio_put(LED_PIN, false);
            busy_wait_ms(100);
        }
    }

    memset(memory, 0, MEM_SIZE);

    uint32_t memory_value = 0;
    uint32_t memory_address = 0;
    while (true) {
        int scanf_result = scanf("%x %x", &memory_value, &memory_address);

        if (scanf_result == 2 && memory_value == UINT32_MAX && memory_address == UINT32_MAX) {
            break;
        }

        if (scanf_result != 2 || ((memory_address > (MEM_SIZE - 4)) && (memory_address != UINT32_MAX))) {
            // Invalid input or address out of bounds, reset to bootloader mode
            reset_usb_boot(1 << LED_PIN, 0);
        }

	if (memory_value == 1 && memory_address == UINT32_MAX) {
            printf("%08x" CRLF, EMULATOR_VERSION);
	    continue;
	}

        // Send checksum of value and address back for verification
        printf("%08x" CRLF, memory_value ^ memory_address);

        // Store the 32-bit value into memory at the specified address
        memory[memory_address] = memory_value & 0xff;
        memory[memory_address + 1] = (memory_value >> 8) & 0xff;
        memory[memory_address + 2] = (memory_value >> 16) & 0xff;
        memory[memory_address + 3] = (memory_value >> 24) & 0xff;
    }
    memory_ready = true;
    multicore_fifo_push_blocking(0);
    sleep_ms(1);
    multicore_fifo_pop_blocking();  // Wait for CPU initialization

    char command_input[32];

    i8080* cpu_ptr = &cpu;

    puts("Memory programming complete. Type 'help' to see available commands.");

    while (true) {
        int scanf_result = scanf("%30s", command_input);
        if (scanf_result != 1) {
            scanf("%*s");  // Clear invalid input
            continue;
        }
        size_t input_length = strlen(command_input);
        if (input_length == 4 && strcmp(command_input, "help") == 0) {
            puts("----------  HELP  ----------");
            puts("Available commands:");
            puts("r         - Reset the CPU without starting code execution.");
            puts("s         - Step the CPU by one instruction and display status (execution remains halted).");
            puts("c         - Continue CPU code execution.");
            puts("m_aaaa    - Halt execution and display 256 bytes of memory starting at address 'aaaa' (hex).");
            puts("po        - Display output port values without halting execution.");
            puts("pi        - Display input port values without halting execution.");
            puts("pi_aa_dd  - Set input port 'aa' (hex) to value 'dd' (hex).");
            puts("i_cc      - Trigger an interrupt with opcode 'cc' (hex).");
            puts("i_rn      - Trigger an interrupt with 'RST n' instruction (n is a hex digit).");
            puts("md_n      - Set CPU T-state delay multiplier to 'n' (default is 1, 1us per T-state).");
            puts("b         - Enter bootloader mode.");
            puts("R         - Restart the program.");
            puts("----------  HELP  ----------");
            continue;
        }
        if (input_length == 1) {  // Single-character commands: r, s, c, b, R
            if (!cpu_stopped) {
                multicore_fifo_push_blocking(0);  // Signal to halt execution
                sleep_ms(1);
                puts("Code execution halted. Current CPU status:");
                i8080_debug_output(cpu_ptr, true);
            }
            switch (command_input[0]) {
                case 'r':
                    reset_cpu(cpu_ptr);
                    i8080_debug_output(cpu_ptr, true);
                    break;
                case 's':
                    handle_intr(cpu_ptr);
                    i8080_step(cpu_ptr);
                    i8080_debug_output(cpu_ptr, true);
                    break;
                case 'c':
                    puts("Resuming CPU execution.");
                    multicore_fifo_push_blocking(0);  // Signal to resume execution
                    sleep_ms(1);
                    break;
                case 'b':
                    multicore_reset_core1();
                    reset_usb_boot(1 << LED_PIN, 0);
                    break;
                case 'R':
                    puts("Restarting program.");
                    goto START_OVER;
                    break;
                default:
                    puts("Invalid command. Type 'help' to see available commands.");
                    break;
            }
        } else if (input_length == 6 && command_input[0] == 'm' && command_input[1] == '_') {  // m_aaaa command
            uint32_t address;
            int sscanf_result = sscanf(command_input, "m_%x", &address);
            if (sscanf_result != 1 || address > (MEM_SIZE - 256)) {
                puts("Invalid memory address. Continuing CPU execution.");
                continue;
            }
            if (!cpu_stopped) {
                multicore_fifo_push_blocking(0);
                sleep_ms(1);
                puts("Code execution halted. Current CPU status:");
                i8080_debug_output(cpu_ptr, true);
            }
            puts("----------  MEMORY DUMP  ----------");
            for (int i = 0; i < 16; i++) {
                int offset = i << 4;
                printf("%04x: ", address + offset);
                for (int j = 0; j < 8; j++) {
                    printf("%02x ", memory[address + offset + j]);
                }
                printf("-- ");
                for (int j = 8; j < 16; j++) {
                    printf("%02x ", memory[address + offset + j]);
                }
                puts("");
            }
            puts("----------  MEMORY DUMP  ----------");
        } else if (input_length == 8 && command_input[0] == 'p' && command_input[1] == 'i') {  // pi_aa_dd command
            uint8_t port_address, port_data;
            int sscanf_result = sscanf(command_input, "pi_%hhx_%hhx", &port_address, &port_data);
            if (sscanf_result != 2) {
                puts("Invalid 'pi' command format. Use 'pi_aa_dd' where 'aa' is port address and 'dd' is data.");
                continue;
            }
            printf("Setting input port %02hhx to value %02hhx" CRLF, port_address, port_data);
            in_port[port_address] = port_data;
        } else if (input_length == 2 && command_input[0] == 'p' && command_input[1] == 'o') {  // po command
            puts("----------  OUTPUT PORTS  ----------");
            for (int i = 0; i < 16; i++) {
                int base_index = i << 4;
                printf("%02x: ", base_index);
                for (int j = 0; j < 8; j++) {
                    printf("%02x ", out_port[base_index + j]);
                }
                printf("-- ");
                for (int j = 8; j < 16; j++) {
                    printf("%02x ", out_port[base_index + j]);
                }
                puts("");
            }
            puts("----------  OUTPUT PORTS  ----------");
        } else if (input_length == 2 && command_input[0] == 'p' && command_input[1] == 'i') {  // pi command
            puts("----------  INPUT PORTS  ----------");
            for (int i = 0; i < 16; i++) {
                int base_index = i << 4;
                printf("%02x: ", base_index);
                for (int j = 0; j < 8; j++) {
                    printf("%02x ", in_port[base_index + j]);
                }
                printf("-- ");
                for (int j = 8; j < 16; j++) {
                    printf("%02x ", in_port[base_index + j]);
                }
                puts("");
            }
            puts("----------  INPUT PORTS  ----------");
        } else if (input_length == 4 && command_input[0] == 'i' && command_input[1] == '_') {  // i_cc or i_rn command
            uint8_t opcode;
            int sscanf_result;
            if (command_input[2] == 'r') {
                sscanf_result = sscanf(command_input, "i_r%hhx", &opcode);
                opcode = 0b11000111 | (opcode << 3);  // Construct RST n opcode
            } else {
                sscanf_result = sscanf(command_input, "i_%hhx", &opcode);
            }
            if (sscanf_result != 1) {
                puts("Invalid interrupt opcode. Continuing.");
                continue;
            }
            printf("Triggering interrupt with opcode %02hhx\n", opcode);
            intr_status = INTR_EN | ((uint32_t)opcode);
        } else if (input_length >= 4 && command_input[0] == 'm' && command_input[1] == 'd' && command_input[2] == '_') {  // md_n command
            uint32_t delay_multiplier;
            int sscanf_result = sscanf(command_input, "md_%u", &delay_multiplier);
            if (sscanf_result != 1) {
                puts("Invalid T-state delay multiplier. Continuing.");
                continue;
            }
            printf("Setting T-state delay multiplier to %u" CRLF, delay_multiplier);
            delay_mlt = delay_multiplier;
        } else {
            puts("Invalid command. Type 'help' to see available commands.");
        }
    }
}

