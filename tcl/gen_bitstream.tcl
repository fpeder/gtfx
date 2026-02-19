open_project ./build/[lindex $argv 0]/[lindex $argv 0].xpr

# ---------------------------------------------------------
# RESET RUNS if they already exist
# ---------------------------------------------------------
# This clears the previous synthesis/implementation state
reset_run synth_1
reset_run impl_1

# ---------------------------------------------------------
# Run Synthesis
# ---------------------------------------------------------
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# Check if synthesis failed
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed"
    exit 1
}

# ---------------------------------------------------------
# Run Implementation & Bitstream
# ---------------------------------------------------------
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# Check if implementation failed
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "ERROR: Implementation failed"
    exit 1
}

puts "--- Bitstream Generated Successfully ---"
