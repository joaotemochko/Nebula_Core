# =============================================================================
# Makefile — simulador Verilator do Nebula Core
#   make            -> build/obj_dir/Vnebula_core_full (sem co-simulação)
#   make COSIM=1    -> com co-simulação lockstep Spike (libriscv)
# =============================================================================
ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
RTL       := $(ROOT)/rtl
RISCV     ?= /opt/riscv
VERILATOR ?= $(RISCV)/bin/verilator
COSIM     ?= 0
OUT       ?= $(ROOT)/build/sim$(if $(filter 1,$(COSIM)),_cosim,)

SRCS := $(RTL)/common/nebula_pkg.sv \
        $(RTL)/common/nebula_csr.sv \
        $(RTL)/frontend/nebula_rvc_expander.sv \
        $(RTL)/frontend/nebula_decoder.sv \
        $(RTL)/frontend/nebula_fetch.sv \
        $(RTL)/cache/nebula_icache.sv \
        $(RTL)/memory/nebula_tlb.sv \
        $(RTL)/memory/nebula_ptw.sv \
        $(RTL)/backend/nebula_lsu.sv \
        $(RTL)/backend/nebula_div.sv \
        $(RTL)/fpu/nebula_fpu.sv \
        $(RTL)/core/nebula_core_full.sv

CPP := $(abspath nebula_sim.cpp)
ifeq ($(COSIM),1)
CPP    += $(abspath cosim_spike.cpp)
CFLAGS := -O2 -std=c++17 -I$(RISCV)/include -DCOSIM
LDFLAGS:= -L$(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -lriscv -lsoftfloat -ldisasm -lfesvr -ldl -lpthread
else
CPP    += $(abspath cosim_stub.cpp)
CFLAGS := -O2 -std=c++17
LDFLAGS:= -lpthread
endif

VFLAGS := --cc --exe --build -j 16 -O3 --x-assign fast --x-initial fast \
          --top-module nebula_core_full -Wno-fatal -Wno-WIDTH -Wno-UNUSED \
          -Wno-UNOPTFLAT -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-CASEINCOMPLETE \
          -CFLAGS "$(CFLAGS)" -LDFLAGS "$(LDFLAGS)" --Mdir $(OUT)

all: $(OUT)/Vnebula_core_full

$(OUT)/Vnebula_core_full: $(SRCS) $(CPP) cosim.h
	$(VERILATOR) $(VFLAGS) $(SRCS) $(CPP)

lint:
	$(VERILATOR) --lint-only -Wall --top-module nebula_core_full $(SRCS)

clean:
	rm -rf $(ROOT)/build/sim $(ROOT)/build/sim_cosim

.PHONY: all lint clean
