# 1. Disable Automatic Hierarchy Update (Manual Mode)
# This prevents the [filemgmt 20-742] warning
set project_file "./build/gtfx/gtfx.xpr"
# Open the project before doing anything else
if {[current_project -quiet] eq ""} {
    open_project $project_file
}

set_property source_mgmt_mode DisplayOnly [current_project]

set sim_dir "./sim"
set tb_files [glob -nocomplain "$sim_dir/*axi*{v,sv}"]

foreach tb_path $tb_files {
    set tb_name [file rootname [file tail $tb_path]]

    puts ">>> Setting Top to: $tb_name"

    # 2. Force the top module
    set_property top $tb_name [get_filesets sim_1]
    
    # 3. Even in manual mode, we tell Vivado to refresh its view
    update_compile_order -fileset sim_1

    # 4. Launch Simulation
    # Use -notrace to keep the console clean
    launch_simulation -simset sim_1 -mode behavioral 
    
    run all
    close_sim
}

# 5. Optional: Switch back to Automatic mode when finished
set_property source_mgmt_mode All [current_project]
