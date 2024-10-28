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
  if(addr == IN_ADDR_PORT1) {
    uint8_t data = (gpio_get_all() & IN_PIN_MSK_PORT1) >> IN_PIN_BASE_PORT1;
    in_port[addr] = data;
    return data;
  } else {
    return in_port[addr];
  }
}

void port_out(void* user_data, uint8_t addr, uint8_t data) {
  if(addr == OUT_ADDR_PORT1) {
    gpio_put_masked(OUT_PIN_MSK_PORT1, data << OUT_PIN_BASE_PORT1);
  } else if(addr == OUT_ADDR_PORT2) {
    gpio_put_masked(OUT_PIN_MSK_PORT2, data << OUT_PIN_BASE_PORT2);
  }
  out_port[addr] = data;
}

inline void handle_intr(i8080 *const c) {
  if(intr_status & INTR_EN) {
    intr_status = intr_status & INTR_MSK;
    i8080_interrupt(c, (uint8_t)(intr_status & INTR_MSK));
  }
}

void reset_cpu(i8080 *const c) {
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
  multicore_fifo_pop_blocking(); // wait for memory programming

  uint32_t curr_time = 0;
  uint32_t target = 0;
  uint64_t prev_cyc = 0;

  i8080* c = &cpu;
  reset_cpu(c);

  multicore_fifo_push_blocking(1);
  sleep_ms(2);

  while(true){
    while((curr_time = time_us_32()) < target){
      tight_loop_contents();
    }
    if(multicore_fifo_rvalid()) {
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
    if(target < curr_time) { // time overflow
      // This happens every ~70 minutes. At most one additional cycle happens every 70 minutes.
      busy_wait_us(delay_mlt * (c->cyc - prev_cyc)); // This CPU cycle might take a little longer.
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
  if(memory == NULL) {
    while(true) {
      gpio_put(LED_PIN, true);
      busy_wait_ms(100);
      gpio_put(LED_PIN, false);
      busy_wait_ms(100);
    }
  }

  memset(memory, 0, MEM_SIZE);

  uint32_t mem_val = 0;
  uint32_t mem_addr = 0;
  while(true) {
    int read_result = scanf("%x %x", &mem_val, &mem_addr);

    if(read_result == 2 && mem_val == UINT32_MAX && mem_addr == UINT32_MAX){
      break;
    }

    if(read_result != 2 || mem_addr > (MEM_SIZE-4)) {
      reset_usb_boot(1 << LED_PIN, 0);
    }

    printf("%08x" CRLF, mem_val ^ mem_addr);

    memory[mem_addr] = mem_val & 0xff;
    memory[mem_addr+1] = (mem_val >> 8) & 0xff;
    memory[mem_addr+2] = (mem_val >> 16) & 0xff;
    memory[mem_addr+3] = (mem_val >> 24) & 0xff;
  }
  memory_ready = true;
  multicore_fifo_push_blocking(0);
  sleep_ms(1);
  multicore_fifo_pop_blocking(); // wait for cpu initialization

  char input[32];

  i8080* c = &cpu;

  puts("Memory Programming Done. Type 'help' to see available options");

  while(true) {
    int result = scanf("%30s", input);
    if(result != 1) {
      scanf("%*s");
      continue;
    }
    size_t len = strlen(input);
    if(len == 4 && strcmp(input, "help") == 0) {
      puts("----------  HELP  ----------");
      puts("HELP: The following commands are available");
      puts("r         - Resets the CPU without starting code execution");
      puts("s         - Steps the CPU for one instruction and prints its status without resuming execution afterwards");
      puts("c         - Makes the CPU resume code execution");
      puts("m_aaaa    - Stops code execution and prints the first 256 bytes of memory after the specified hex address");
      puts("po        - Reads all of the output ports without stopping code execution");
      puts("pi        - Reads the last value of input ports without stopping code execution");
      puts("pi_aa_dd  - Sets the value of input port 'aa' to 'dd'");
      puts("i_cc      - Make the interrupt pin go high and set the interrupt opcode to 'cc'");
      puts("i_rn      - Make the interrupt pin go high and set the interrupt opcode to 'RST n'");
      puts("md_n      - Set CPU T-state delay multiplier to 'n'. By default it is '1 * 1us per T-state.'");
      puts("b         - Sends the board into bootloader mode");
      puts("R         - Restarts the board");
      puts("----------  HELP  ----------");
      continue;
    }
    if(len == 1) { //r, c, s commands
      if(!cpu_stopped){
        multicore_fifo_push_blocking(0);
        sleep_ms(1);
        puts("Stopped code execution. The current status is:");
        i8080_debug_output(c, true);
      }
      if(input[0] == 'r') {
        reset_cpu(c);
        i8080_debug_output(c, true);
      } else if(input[0] == 's') {
        handle_intr(c);
        i8080_step(c);
        i8080_debug_output(c, true);
      } else if(input[0] == 'c') {
        puts("Resuming normal CPU operations.");
        multicore_fifo_push_blocking(0);
        sleep_ms(1);
      } else if(input[0] == 'b') {
        multicore_reset_core1();
        reset_usb_boot(1 << LED_PIN, 0);
      } else if(input[0] == 'R') {
        puts("Restarting code.");
        goto START_OVER;
      }
    } else if(len == 6 && input[0] == 'm' && input[1] == '_') { // m_aaaa command
      uint32_t addr;
      int result = sscanf(input, "m_%x", &addr);
      if(result != 1 || addr > (MEM_SIZE - 256)){
        puts("Invalid memory start address. Continuing CPU code execution.");
        continue;
      }
      if(!cpu_stopped){
        multicore_fifo_push_blocking(0);
        sleep_ms(1);
        puts("Stopped code execution. The current status is:");
        i8080_debug_output(c, true);
      }
      puts("----------  MEM  ----------");
      for(int i = 0; i < 16; i++) {
        int bi = i << 4;
        printf("%04x: ", addr + bi);
        for(int j = 0; j < 8; j++) {
          printf("%02x ", memory[addr + bi + j]);
        }
        printf("-- ");
        for(int j = 8; j < 16; j++) {
          printf("%02x ", memory[addr + bi + j]);
        }
        puts("");
      }
      puts("----------  MEM  ----------");
    } else if(len == 8 && input[0] == 'p' && input[1] == 'i') { // pi_aa_dd command
      uint8_t addr, data;
      int result = sscanf(input, "pi_%hhx_%hhx", &addr, &data);
      if(result != 2) {
        puts("Invalid 'pi' command. Check address and data values.");
        continue;
      }
      printf("Setting input port %02hhx value to %02hhx" CRLF, addr, data);
      in_port[addr] = data;
    } else if(len == 2 && input[0] == 'p' && input[1] == 'o') { // po command
      puts("----------  PO  ----------");
      for(int i = 0; i < 16; i++) {
        int bi = i << 4;
        printf("%02x: ", bi);
        for(int j = 0; j < 8; j++) {
          printf("%02x ", out_port[bi + j]);
        }
        printf("-- ");
        for(int j = 8; j < 16; j++) {
          printf("%02x ", out_port[bi + j]);
        }
        puts("");
      }
      puts("----------  PO  ----------");
    } else if(len == 2 && input[0] == 'p' && input[1] == 'i') { // pi command
      puts("----------  PI  ----------");
      for(int i = 0; i < 16; i++) {
        int bi = i << 4;
        printf("%02x: ", bi);
        for(int j = 0; j < 8; j++) {
          printf("%02x ", in_port[bi + j]);
        }
        printf("-- ");
        for(int j = 8; j < 16; j++) {
          printf("%02x ", in_port[bi + j]);
        }
        puts("");
      }
      puts("----------  PI  ----------");
    } else if(len == 4 && input[0] == 'i' && input[1] == '_') { // i_cc command
      uint8_t opcode;
      int result;
      if(input[2] == 'r') {
        result = sscanf(input, "i_r%hhx", &opcode);
        opcode = 0b11000111 | (opcode << 3);
      } else {
        result = sscanf(input, "i_%hhx", &opcode);
      }
      if(result != 1) {
        puts("Invalid interrupt opcode. Continuing.");
        continue;
      }
      printf("Setting interrupt, with opcode %02hhx\n", opcode);
      intr_status = INTR_EN | ((uint32_t)opcode);
    } else if(len >= 4 && input[0] == 'm' && input[1] == 'd' && input[2] == '_') {
      uint32_t mlt_tmp;
      int result;
      result = sscanf(input, "md_%u", &mlt_tmp);
      if(result != 1) {
        puts("Invalid T-state delay multiplier. Continuing.");
        continue;
      } 
      printf("Setting speed multiplier to %u" CRLF, mlt_tmp);
      delay_mlt = mlt_tmp;
    } else {
      puts("Invalid command. Type 'help' to see available commands.");
    }
  }
}
