# run_sim.tcl — Simulation Runner with pass/fail tracking
# Usage: vivado -mode batch -source run_sim.tcl -tclargs <project_name> [tb_filter]

# ---------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------
set proj_name [lindex $argv 0]
set tb_filter ""
if {[llength $argv] >= 2} {
    set tb_filter [lindex $argv 1]
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

set_property source_mgmt_mode DisplayOnly [current_project]

# ---------------------------------------------------------
# Discover testbenches
# ---------------------------------------------------------
set sim_dir "./sim"
set all_tb_files [glob -nocomplain "$sim_dir/tb_*{v,sv}"]
set all_tb_names {}
foreach f $all_tb_files {
    lappend all_tb_names [file rootname [file tail $f]]
}

# ---------------------------------------------------------
# Apply filter
# ---------------------------------------------------------
if {$tb_filter ne ""} {
    # Support both "tb_chorus" and "chorus" as filter
    set filter_name $tb_filter
    if {![string match "tb_*" $filter_name]} {
        set filter_name "tb_${filter_name}"
    }

    set matched {}
    foreach tb $all_tb_names {
        if {[string match "*${filter_name}*" $tb]} {
            lappend matched $tb
        }
    }

    if {[llength $matched] == 0} {
        puts "ERROR: No testbench matching '$tb_filter'"
        puts "Available testbenches:"
        foreach tb [lsort $all_tb_names] {
            puts "  $tb"
        }
        exit 1
    }
    set tb_list $matched
} else {
    set tb_list $all_tb_names
}

set tb_list [lsort $tb_list]
set n_total [llength $tb_list]
puts "=== Running $n_total testbench(es) ==="
puts ""

# ---------------------------------------------------------
# Run each testbench
# ---------------------------------------------------------
set pass_list {}
set fail_list {}
set t_total [clock seconds]

foreach tb_name $tb_list {
    puts ">>> \[$tb_name\]"
    set t0 [clock seconds]

    set_property top $tb_name [get_filesets sim_1]
    update_compile_order -fileset sim_1

    set sim_ok 1
    set fail_reason ""

    # launch_simulation + run — $fatal triggers a Tcl error caught by catch
    if {[catch {launch_simulation -simset sim_1 -mode behavioral} launch_err]} {
        set sim_ok 0
        set fail_reason "launch failed: $launch_err"
    } else {
        if {[catch {run all} run_err]} {
            set sim_ok 0
            set fail_reason "runtime error (likely \$fatal): $run_err"
        }

        # Scan simulation log for FAIL markers
        if {$sim_ok} {
            set log_path "./build/${proj_name}/${proj_name}.sim/sim_1/behav/xsim/simulate.log"
            if {[file exists $log_path]} {
                set fp [open $log_path r]
                set log_content [read $fp]
                close $fp
                if {[regexp -nocase {FAIL} $log_content]} {
                    set sim_ok 0
                    set fail_reason "FAIL detected in simulation log"
                }
            }
        }

        catch {close_sim}
    }

    set t1 [clock seconds]
    set elapsed [fmt_elapsed $t0 $t1]

    if {$sim_ok} {
        puts "    PASS ($elapsed)"
        lappend pass_list $tb_name
    } else {
        puts "    FAIL ($elapsed) — $fail_reason"
        lappend fail_list $tb_name
    }
    puts ""
}

# ---------------------------------------------------------
# Restore project mode
# ---------------------------------------------------------
set_property source_mgmt_mode All [current_project]

# ---------------------------------------------------------
# Summary
# ---------------------------------------------------------
set t_end [clock seconds]
set n_pass [llength $pass_list]
set n_fail [llength $fail_list]

puts "==========================================="
puts "  Simulation Summary"
puts "==========================================="
puts "  Total:  $n_total"
puts "  Passed: $n_pass"
puts "  Failed: $n_fail"
puts "  Time:   [fmt_elapsed $t_total $t_end]"
puts "-------------------------------------------"

if {$n_fail > 0} {
    puts "  FAILED:"
    foreach tb $fail_list {
        puts "    - $tb"
    }
}
if {$n_pass > 0} {
    puts "  PASSED:"
    foreach tb $pass_list {
        puts "    - $tb"
    }
}
puts "==========================================="

if {$n_fail > 0} {
    exit 1
}
