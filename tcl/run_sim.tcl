open_project ./build/[lindex $argv 0]/[lindex $argv 0].xpr
launch_simulation
# Run for a specific time or until finished
run 100us 
# Optional: close_project
