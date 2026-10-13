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
NCORES    ?= 1
OUT       ?= $(ROOT)/build/sim$(if $(filter 1,$(COSIM)),_cosim,)$(if $(filter-out 1,$(NCORES)),_smp$(NCORES),)

SRCS := $(RTL)/common/nebula_pkg.sv \
        $(RTL)/common/nebula_csr.sv \
        $(RTL)/frontend/nebula_rvc_expander.sv \
        $(RTL)/frontend/nebula_decoder.sv \
        $(RTL)/frontend/nebula_fetch.sv \
        $(RTL)/cache/nebula_icache.sv \
        $(RTL)/memory/nebula_tlb.sv \
        $(RTL)/memory/nebula_pma.sv \
        $(RTL)/memory/nebula_pmp.sv \
        $(RTL)/memory/nebula_ptw.sv \
        $(RTL)/backend/nebula_lsu.sv \
        $(RTL)/backend/nebula_div.sv \
        $(RTL)/fpu/nebula_fpu.sv \
        $(RTL)/core/nebula_core_full.sv \
        $(RTL)/cluster/nebula_interconnect.sv \
        $(RTL)/cluster/nebula_cluster.sv

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

CFLAGS += $(EXTRA_CFLAGS) -DNCORES=$(NCORES)
PROF   ?= 0
FST    ?= 0
ifeq ($(FST),1)
PROF_VFLAGS += --trace-fst --trace-structs
endif
ifeq ($(PROF),1)
CFLAGS  += -pg
LDFLAGS += -pg
PROF_VFLAGS := --prof-cfuncs
endif
VFLAGS := --cc --exe --build -j 16 -O3 $(THREADS_FLAGS) --x-assign fast --x-initial fast \
          --top-module nebula_cluster -GNUM_CORES=$(NCORES) -Wno-fatal -Wno-WIDTH -Wno-UNUSED \
          -Wno-UNOPTFLAT -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-CASEINCOMPLETE \
          $(PROF_VFLAGS) -CFLAGS "$(CFLAGS)" -LDFLAGS "$(LDFLAGS)" --Mdir $(OUT)

all: $(OUT)/Vnebula_cluster

$(OUT)/Vnebula_cluster: $(SRCS) $(CPP) cosim.h
	$(VERILATOR) $(VFLAGS) $(SRCS) $(CPP)

lint:
	$(VERILATOR) --lint-only -Wall --top-module nebula_cluster $(SRCS)

clean:
	rm -rf $(ROOT)/build/sim $(ROOT)/build/sim_cosim $(ROOT)/build/sim_smp*

.PHONY: all lint clean

# -----------------------------------------------------------------------------
# Top AXI4 (nebula_axi_top) com escravo AXI em C++ — valida a ponte AXI
#   make axi [NCORES=n]
# -----------------------------------------------------------------------------
AXI_OUT ?= $(ROOT)/build/sim_axi$(if $(filter-out 1,$(NCORES)),_smp$(NCORES),)
AXI_SRCS := $(SRCS) $(RTL)/wrapper/nebula_axi_bridge.sv $(RTL)/wrapper/nebula_axi_top.sv
AXI_PMA  := -GPMA_BASE="448'h80000000000000100000000000000c000000000000020000000000000010000000000000001000" -GPMA_SIZE="448'h100000000000000000010000000000400000000000000100000000000000100000000000001000" -GPMA_ATTR="24'h3924d"

axi: $(AXI_OUT)/Vnebula_axi_top

$(AXI_OUT)/Vnebula_axi_top: $(AXI_SRCS) nebula_axi_sim.cpp soc_model.h
	$(VERILATOR) --cc --exe --build -j 16 -O3 --x-assign fast --x-initial fast \
	  --top-module nebula_axi_top -GNUM_CORES=$(NCORES) -GTIMER_DIV=100 $(AXI_PMA) \
	  -Wno-fatal -Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY \
	  -Wno-CASEINCOMPLETE -CFLAGS "-O2 -std=c++17 -DNCORES=$(NCORES)" -LDFLAGS -lpthread \
	  --Mdir $(AXI_OUT) $(AXI_SRCS) $(abspath nebula_axi_sim.cpp)

.PHONY: axi
