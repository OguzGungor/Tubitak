@echo off
REM ****************************************************************************
REM Vivado (TM) v2020.2 (64-bit)
REM
REM Filename    : simulate.bat
REM Simulator   : Xilinx Vivado Simulator
REM Description : Script for simulating the design by launching the simulator
REM
REM Generated by Vivado on Mon Feb 08 04:32:34 +0300 2021
REM SW Build 3064766 on Wed Nov 18 09:12:45 MST 2020
REM
REM Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
REM
REM usage: simulate.bat
REM
REM ****************************************************************************
REM simulate design
echo "xsim top_test_behav -key {Behavioral:sim_1:Functional:top_test} -tclbatch top_test.tcl -view C:/Users/moguz/OneDrive/Vivado/graph_proc_ver0-20210128T235715Z-001/graph_proc_ver0/top_test_behav.wcfg -log simulate.log"
call xsim  top_test_behav -key {Behavioral:sim_1:Functional:top_test} -tclbatch top_test.tcl -view C:/Users/moguz/OneDrive/Vivado/graph_proc_ver0-20210128T235715Z-001/graph_proc_ver0/top_test_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
