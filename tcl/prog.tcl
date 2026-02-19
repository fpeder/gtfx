# Open the hardware manager
open_hw_manager

# Connect to the local hw_server
connect_hw_server -url localhost:3121
refresh_hw_server

# Find the target (Arty A7 is an Artix-7 device)
set target_hw_devices [get_hw_targets]
if { [llength $target_hw_devices] == 0 } {
    puts "Error: No JTAG targets found. Is your Arty A7 plugged in?"
    exit 1
}

open_hw_target

# Select the first device on the chain (the xc7a100t)
set device [lindex [get_hw_devices] 0]
current_hw_device $device
refresh_hw_device $device

# Path to the bitstream (passed as an argument)
set bitstream_path [lindex $argv 1]

# Set the bitstream and program
set_property PROGRAM.FILE $bitstream_path $device
program_hw_devices $device

puts "--- Programming Complete! ---"
close_hw_target
disconnect_hw_server
close_hw_manager
