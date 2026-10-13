# =============================================================================
# Build do Embench IoT para o Nebula Core (RV64GC, bare-metal, M-mode)
#   make            -> compila todos os benchmarks em ../../build/embench/<bench>.elf
#   make GSF=n      -> GLOBAL_SCALE_FACTOR
# =============================================================================
ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
EMB     := $(ROOT)/benchmarks/embench-iot
BSP     := $(ROOT)/benchmarks/nebula
OUT     := $(ROOT)/build/embench
RISCV   ?= /opt/riscv
CC      := $(RISCV)/bin/riscv64-unknown-elf-gcc
GSF     ?= 1
WARMUP  ?= 1
ARCH    ?= -march=rv64imafdc_zicsr_zifencei -mabi=lp64d
CFLAGS  := $(ARCH) -mcmodel=medany -O2 -std=gnu17 -ffunction-sections -fdata-sections \
           -DGLOBAL_SCALE_FACTOR=$(GSF) -DWARMUP_HEAT=$(WARMUP) -DHAVE_BOARDSUPPORT_H \
           -I$(EMB)/support -I$(BSP) -w
LDFLAGS := $(ARCH) -mcmodel=medany -nostartfiles -T$(BSP)/link.ld -Wl,--gc-sections

BENCHES := $(notdir $(wildcard $(EMB)/src/*))
ELFS    := $(addprefix $(OUT)/,$(addsuffix .elf,$(BENCHES)))

all: $(ELFS)

$(OUT)/%.elf: $(EMB)/src/%/*.c $(BSP)/boardsupport.c $(BSP)/crt0.S $(BSP)/link.ld
	@mkdir -p $(OUT)
	$(CC) $(CFLAGS) -I$(EMB)/src/$* $(BSP)/crt0.S $(BSP)/boardsupport.c \
	   $(EMB)/support/main.c $(EMB)/support/beebsc.c $(wildcard $(EMB)/src/$*/*.c) \
	   $(LDFLAGS) -lm -lc -lgcc -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $(OUT)/$*.dump

clean:
	rm -rf $(OUT)

.PHONY: all clean
