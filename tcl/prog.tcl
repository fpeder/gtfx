# prog.tcl — FPGA Programming
# Usage: vivado -mode batch -source prog.tcl -tclargs <project_name> <bitstream_path> [hw_server_url]

# ---------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------
set proj_name [lindex $argv 0]
set bitstream_path [lindex $argv 1]
set hw_server_url "localhost:3121"
if {[llength $argv] >= 3 && [lindex $argv 2] ne ""} {
    set hw_server_url [lindex $argv 2]
}

# ---------------------------------------------------------
# Cleanup proc
# ---------------------------------------------------------
proc cleanup {} {
    catch {close_hw_target}
    catch {disconnect_hw_server}
    catch {close_hw_manager}
}

# ---------------------------------------------------------
# Check bitstream exists
# ---------------------------------------------------------
if {$bitstream_path eq "" || ![file exists $bitstream_path]} {
    puts "ERROR: Bitstream not found: '$bitstream_path'"
    puts "       Have you run 'make bitstream' first?"
    exit 1
}
puts "Bitstream: $bitstream_path ([format "%.1f" [expr {[file size $bitstream_path] / 1024.0}]] KB)"

# ---------------------------------------------------------
# Connect to hardware server
# ---------------------------------------------------------
open_hw_manager

if {[catch {connect_hw_server -url $hw_server_url} err]} {
    puts "ERROR: Could not connect to hw_server at '$hw_server_url'"
    puts "       Is hw_server running? Try: hw_server &"
    puts "       ($err)"
    cleanup
    exit 1
}
refresh_hw_server

# ---------------------------------------------------------
# Find JTAG target
# ---------------------------------------------------------
set target_hw_devices [get_hw_targets]
if {[llength $target_hw_devices] == 0} {
    puts "ERROR: No JTAG targets found. Is the board plugged in?"
    cleanup
    exit 1
}

open_hw_target

set device [lindex [get_hw_devices] 0]
current_hw_device $device
refresh_hw_device $device

puts "Device: [get_property PART [current_hw_device]]"

# ---------------------------------------------------------
# Program
# ---------------------------------------------------------
set_property PROGRAM.FILE $bitstream_path $device

if {[catch {program_hw_devices $device} err]} {
    puts "ERROR: Programming failed"
    puts "       $err"
    cleanup
    exit 1
}

puts "--- Programming Complete! ---"
cleanup
