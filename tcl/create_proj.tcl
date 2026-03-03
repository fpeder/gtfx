# 1. Project Configuration
set project_name "gtfx"
set build_dir    "./build"
set target_part  "xc7a100tcsg324-1"

# 2. Create the Project
# -force ensures it overwrites the old build directory
create_project $project_name $build_dir/$project_name -part $target_part -force

puts "--- Generating Clock Wizard IP ---"
create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0

set_property -dict [list \
  CONFIG.PRIM_IN_FREQ {100.000} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {12.288} \
  CONFIG.USE_RESET {true} \
  CONFIG.USE_LOCKED {true} \
  CONFIG.RESET_TYPE {ACTIVE_LOW} \
  CONFIG.RESET_PORT {resetn} \
] [get_ips clk_wiz_0]

# 3. Generate the implementation and simulation files
generate_target all [get_ips clk_wiz_0]
create_ip_run [get_ips clk_wiz_0]
puts "--- Clock Wizard Generated ---"

# ... (rest of script adding design sources)
# 3. Add Design Sources (SystemVerilog)
proc find_sv {dir} {
    set result {}
    foreach f [glob -nocomplain -directory $dir *.sv] {
        lappend result $f
    }
    foreach subdir [glob -nocomplain -directory $dir -type d *] {
        set result [concat $result [find_sv $subdir]]
    }
    return $result
}

add_files -fileset sources_1 [find_sv ./src]

# 4. Add Simulation Sources (Testbenches)
add_files -fileset sim_1 [glob ./sim/*.sv]

# 5. Add Constraints (XDC)
add_files -fileset constrs_1 [glob ./constrs/*.xdc]

# 6. Handle Block Designs (If one exists)
# This looks for your exported "recipe" and rebuilds the .bd file
if {[file exists "./tcl/recreate_bd.tcl"]} {
    source ./tcl/recreate_bd.tcl

    # Generate the HDL Wrapper so synthesis can "see" the block design
    set bd_file [get_files *.bd]
    generate_target all $bd_file
    make_wrapper -files $bd_file -top
    add_files [glob [file dirname [get_property NAME $bd_file]]/hdl/*_wrapper.v]
}

# 7. Finalize Organization
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "--- Project $project_name Created Successfully ---"
