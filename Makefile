PROJ_NAME = gtfx
SCRIPTS   = ./tcl
BUILD_DIR = ./build

PROJECT_FILE = $(BUILD_DIR)/$(PROJ_NAME)/$(PROJ_NAME).xpr
BIT_FILE = $(BUILD_DIR)/$(PROJ_NAME)/$(PROJ_NAME).runs/impl_1/top.bit

JOBS      ?= 6
TB        ?=
HW_SERVER ?=

.PHONY: all proj bitstream sim prog clean open

all: bitstream

$(PROJECT_FILE):
	mkdir -p $(BUILD_DIR)
	vivado -mode batch -source $(SCRIPTS)/create_proj.tcl -tclargs $(PROJ_NAME)

proj: $(PROJECT_FILE)

bitstream: $(PROJECT_FILE)
	vivado -mode batch -source $(SCRIPTS)/gen_bitstream.tcl -tclargs $(PROJ_NAME) $(JOBS)

sim: $(PROJECT_FILE)
	vivado -mode batch -source $(SCRIPTS)/run_sim.tcl -tclargs $(PROJ_NAME) $(TB)

prog:
	vivado -mode batch -source $(SCRIPTS)/prog.tcl -tclargs $(PROJ_NAME) $(BIT_FILE) $(HW_SERVER)

open: $(PROJECT_FILE)
	vivado $(PROJECT_FILE) &

clean:
	rm -rf $(BUILD_DIR)
	rm -rf *.log *.jou .Xil usage_statistics_webtalk.* *.pb vivado*
