# gen_bitstream.tcl — Synthesis + Implementation + Bitstream
# Usage: vivado -mode batch -source gen_bitstream.tcl -tclargs <project_name> [jobs]

# ---------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------
set proj_name [lindex $argv 0]
set jobs 4
if {[llength $argv] >= 2 && [lindex $argv 1] ne ""} {
    set jobs [lindex $argv 1]
}

set xpr_path "./build/${proj_name}/${proj_name}.xpr"

# ---------------------------------------------------------
# Helper: format elapsed seconds
# ---------------------------------------------------------
proc fmt_elapsed {start end} {
    set secs [expr {int($end - $start)}]
    set m [expr {$secs / 60}]
    set s [expr {$secs % 60}]
    return [format "%d:%02d" $m $s]
}

# ---------------------------------------------------------
# Open project
# ---------------------------------------------------------
if {[catch {open_project $xpr_path} err]} {
    puts "ERROR: Could not open project '$xpr_path'"
    puts "       Have you run 'make proj' first?"
    puts "       ($err)"
    exit 1
}

set t_total [clock seconds]

# ---------------------------------------------------------
# Reset runs
# ---------------------------------------------------------
reset_run synth_1
reset_run impl_1

# ---------------------------------------------------------
# Synthesis
# ---------------------------------------------------------
puts "=== Synthesis (jobs=$jobs) ==="
set t0 [clock seconds]

launch_runs synth_1 -jobs $jobs
wait_on_run synth_1

set t1 [clock seconds]
set synth_status [get_property STATUS [get_runs synth_1]]
set synth_progress [get_property PROGRESS [get_runs synth_1]]

if {$synth_progress != "100%" || [string match "*error*" [string tolower $synth_status]]} {
    puts "ERROR: Synthesis failed (status: $synth_status, progress: $synth_progress)"
    exit 1
}
puts "--- Synthesis complete in [fmt_elapsed $t0 $t1] (status: $synth_status) ---"

# ---------------------------------------------------------
# Implementation + Bitstream
# ---------------------------------------------------------
puts "=== Implementation + Bitstream (jobs=$jobs) ==="
set t2 [clock seconds]

launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1

set t3 [clock seconds]
set impl_status [get_property STATUS [get_runs impl_1]]
set impl_progress [get_property PROGRESS [get_runs impl_1]]

if {$impl_progress != "100%" || [string match "*error*" [string tolower $impl_status]]} {
    puts "ERROR: Implementation failed (status: $impl_status, progress: $impl_progress)"
    exit 1
}
puts "--- Implementation complete in [fmt_elapsed $t2 $t3] (status: $impl_status) ---"

# ---------------------------------------------------------
# Timing summary
# ---------------------------------------------------------
puts ""
puts "=== Timing Summary ==="
open_run impl_1

set wns  [get_property STATS.WNS  [get_runs impl_1]]
set tns  [get_property STATS.TNS  [get_runs impl_1]]
set whs  [get_property STATS.WHS  [get_runs impl_1]]
set ths  [get_property STATS.THS  [get_runs impl_1]]

puts [format "  WNS: %s ns   TNS: %s ns" $wns $tns]
puts [format "  WHS: %s ns   THS: %s ns" $whs $ths]

if {[string is double -strict $wns] && $wns < 0} {
    puts "  WARNING: Setup timing not met (WNS < 0)!"
}
if {[string is double -strict $whs] && $whs < 0} {
    puts "  WARNING: Hold timing not met (WHS < 0)!"
}

# ---------------------------------------------------------
# Utilization summary
# ---------------------------------------------------------
puts ""
puts "=== Utilization Summary ==="
foreach {key label} {
    STATS.UTIL.LUT   "LUTs"
    STATS.UTIL.FF    "FFs"
    STATS.UTIL.BRAM  "BRAMs"
    STATS.UTIL.DSP   "DSPs"
} {
    if {![catch {set val [get_property $key [get_runs impl_1]]}]} {
        puts [format "  %-6s %s" "${label}:" $val]
    }
}

# ---------------------------------------------------------
# Bitstream info
# ---------------------------------------------------------
puts ""
puts "=== Bitstream ==="
set bit_dir "./build/${proj_name}/${proj_name}.runs/impl_1"
set bit_file [glob -nocomplain "$bit_dir/*.bit"]
if {[llength $bit_file] > 0} {
    set bit_file [lindex $bit_file 0]
    set bit_size [file size $bit_file]
    set bit_kb [format "%.1f" [expr {$bit_size / 1024.0}]]
    puts "  File: $bit_file"
    puts "  Size: ${bit_kb} KB"
} else {
    puts "  WARNING: No .bit file found in $bit_dir"
}

set t_end [clock seconds]
puts ""
puts [format "=== Done in %s (synth %s + impl %s) ===" \
    [fmt_elapsed $t_total $t_end] \
    [fmt_elapsed $t0 $t1] \
    [fmt_elapsed $t2 $t3]]
