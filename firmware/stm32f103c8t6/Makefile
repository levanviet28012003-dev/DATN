TARGET := ackermann
BUILD := build
PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
OPENOCD ?= openocd

# Chương trình chính nằm trong core/src; có thể đổi APP_MAIN khi cần.
APP_MAIN ?= core/src/main.c
SOURCE_DIRS := core/src driver/src driver/device/src peripheral/src
C_SOURCES := $(filter-out core/src/main.c peripheral/src/main.c,\
              $(foreach directory,$(SOURCE_DIRS),$(wildcard $(directory)/*.c))) $(APP_MAIN)
ASM_SOURCES := startup/startup_stm32f103c8t6.s
OBJECTS := $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o))
CPU_FLAGS := -mcpu=cortex-m3 -mthumb
INCLUDE_DIRS := core/inc driver/inc driver/device/inc peripheral/inc
CPPFLAGS := $(addprefix -I,$(INCLUDE_DIRS)) -DSTM32F103C8T6 -DSTM32F103xB
CFLAGS := $(CPU_FLAGS) -std=c11 -Os -g3 -Wall -Wextra -Werror \
          -ffreestanding -fno-builtin -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(CPU_FLAGS) -nostdlib -Tlinker/stm32f103c8t6.ld \
           -Wl,--gc-sections,-Map=$(BUILD)/$(TARGET).map,--print-memory-usage

.PHONY: all clean size flash debug-server compile-commands
all: compile-commands $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

# VS Code đọc lệnh biên dịch thực tế, kể cả khi mở từ gốc ackermann_ws.
compile-commands:
	python3 tools/generate_compile_commands.py --compiler "$(CC)" --build-dir "$(BUILD)" \
	  --flags="$(CPPFLAGS) $(CFLAGS)" $(C_SOURCES)

$(BUILD)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CPU_FLAGS) -g3 -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) linker/stm32f103c8t6.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -lgcc -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

# Chỉ nạp khi người dùng chủ động chạy make flash và đã kết nối ST-Link.
flash: $(BUILD)/$(TARGET).elf
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32f1x.cfg \
	  -c "program $< verify reset exit"

debug-server:
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32f1x.cfg

clean:
	$(RM) -r $(BUILD)

-include $(OBJECTS:.o=.d)
