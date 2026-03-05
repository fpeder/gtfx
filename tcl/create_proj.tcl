# create_proj.tcl — Project creation + Clock Wizard IP generation
# Usage: vivado -mode batch -source create_proj.tcl -tclargs [project_name]

# ---------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------
set project_name "gtfx"
if {[llength $argv] >= 1 && [lindex $argv 0] ne ""} {
    set project_name [lindex $argv 0]
}

set build_dir    "./build"
set target_part  "xc7a100tcsg324-1"

set t0 [clock seconds]

# ---------------------------------------------------------
# Create project
# ---------------------------------------------------------
create_project $project_name $build_dir/$project_name -part $target_part -force

# ---------------------------------------------------------
# Generate Clock Wizard IP
# ---------------------------------------------------------
puts "--- Generating Clock Wizard IP ---"
if {[catch {
    create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0

    set_property -dict [list \
      CONFIG.PRIM_IN_FREQ {100.000} \
      CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {12.288} \
      CONFIG.USE_RESET {true} \
      CONFIG.USE_LOCKED {true} \
      CONFIG.RESET_TYPE {ACTIVE_LOW} \
      CONFIG.RESET_PORT {resetn} \
    ] [get_ips clk_wiz_0]

    generate_target all [get_ips clk_wiz_0]
    create_ip_run [get_ips clk_wiz_0]
} ip_err]} {
    puts "ERROR: Clock Wizard IP generation failed"
    puts "       $ip_err"
    exit 1
}
puts "--- Clock Wizard Generated ---"

# ---------------------------------------------------------
# Add Design Sources (SystemVerilog)
# ---------------------------------------------------------
proc find_sv {dir} {
    set result {}
    foreach f [glob -nocomplain -directory $dir *.sv] {
        lappend result $f
    }
    foreach subdir [glob -nocomplain -directory $dir -type d *] {
        if {[file tail $subdir] eq "unused"} continue
        set result [concat $result [find_sv $subdir]]
    }
    return $result
}

set design_files [find_sv ./src]
set n_design [llength $design_files]
if {$n_design > 0} {
    add_files -fileset sources_1 $design_files
}

# ---------------------------------------------------------
# Add Simulation Sources (Testbenches)
# ---------------------------------------------------------
set sim_files [glob -nocomplain ./sim/*.sv]
set n_sim [llength $sim_files]
if {$n_sim > 0} {
    add_files -fileset sim_1 $sim_files
}

# ---------------------------------------------------------
# Add Constraints (XDC)
# ---------------------------------------------------------
set constr_files [glob -nocomplain ./constrs/*.xdc]
set n_constr [llength $constr_files]
if {$n_constr > 0} {
    add_files -fileset constrs_1 $constr_files
}

# ---------------------------------------------------------
# Handle Block Designs (if one exists)
# ---------------------------------------------------------
if {[file exists "./tcl/recreate_bd.tcl"]} {
    source ./tcl/recreate_bd.tcl

    set bd_file [get_files *.bd]
    generate_target all $bd_file
    make_wrapper -files $bd_file -top
    add_files [glob [file dirname [get_property NAME $bd_file]]/hdl/*_wrapper.v]
}

# ---------------------------------------------------------
# Finalize
# ---------------------------------------------------------
set_property top top [current_fileset]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

set t1 [clock seconds]
set elapsed [expr {$t1 - $t0}]

puts ""
puts "=== Project '$project_name' Created ==="
puts "  Design sources: $n_design"
puts "  Sim sources:    $n_sim"
puts "  Constraints:    $n_constr"
puts "  Elapsed:        ${elapsed}s"
puts "========================================="
