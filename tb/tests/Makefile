# Testes dirigidos do Nebula Core (verificados por co-simulação com o Spike)
ROOT  := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
OUT   := $(ROOT)/build/tests
RISCV ?= /opt/riscv
CC    := $(RISCV)/bin/riscv64-unknown-elf-gcc
ARCH  := -march=rv64gc -mabi=lp64d -mcmodel=medany
LDW   := -Wl,--no-warn-rwx-segments

ASM_TESTS := access_fault pmp_rand
C_TESTS   := asid_test:crt_s misaligned_test:crt_s_deleg trigger_test:crt_m

SMP_N := 1 2 4

ELFS := $(addprefix $(OUT)/,$(addsuffix .elf,$(ASM_TESTS))) \
        $(foreach t,$(C_TESTS),$(OUT)/$(word 1,$(subst :, ,$(t))).elf) \
        $(foreach n,$(SMP_N),$(OUT)/smp_test_$(n).elf)

all: $(ELFS)

$(OUT)/%.elf: %.S link.ld
	@mkdir -p $(OUT)
	$(CC) $(ARCH) -nostdlib -T link.ld $< $(LDW) -o $@

define CTEST
$(OUT)/$(1).elf: $(1).c $(2).S link_c.ld
	@mkdir -p $(OUT)
	$(CC) $(ARCH) -O2 -nostartfiles -T link_c.ld $(2).S $(1).c -lc -lgcc $(LDW) -o $$@
endef
$(foreach t,$(C_TESTS),$(eval $(call CTEST,$(word 1,$(subst :, ,$(t))),$(word 2,$(subst :, ,$(t))))))

$(OUT)/smp_test_%.elf: smp_test.c crt_smp.S link_c.ld
	@mkdir -p $(OUT)
	$(CC) $(ARCH) -O2 -DNHARTS=$* -nostartfiles -T link_c.ld crt_smp.S smp_test.c -lc -lgcc $(LDW) -o $@

clean:
	rm -rf $(OUT)
.PHONY: all clean
