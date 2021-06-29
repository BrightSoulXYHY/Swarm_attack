@ECHO OFF

REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Start index of vehicle number (should larger than 0)
REM This option is useful for simulation with multi-computers
SET /a START_INDEX=1


REM Set the start UDP port for SIMULINK/OFFBOARD API
REM This option should not be modified for swarm simulation
SET /a UDP_START_PORT=20100


REM Set use DLL model name or not, use number index or name string
REM This option is useful for simulation with other types of vehicles instead of multicopters
set DLLModel=0

REM Set the simulation mode on CopterSim, use number index or name string
REM e.g., SimMode=2 equals to  SimMode=PX4_SITL_RFLY
set SimMode=2


REM Set the map, use index or name of the map on CopterSim
REM e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
SET UE4_MAP=OldFactory

REM Set the origin x,y position (m) and yaw angle (degree) at the map
SET /a ORIGIN_POS_X=-188
SET /a ORIGIN_POS_Y=-192
SET /a ORIGIN_YAW=85.234

REM Set the interval between two vehicle, unit:m 
SET /a VEHICLE_INTERVAL=4


REM Set broadcast to other computer; 0: only this computer, 1: broadcast; or use IP address to increase speed
REM e.g., IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
rem SET IS_BROADCAST=192.168.204.140
rem SET IS_BROADCAST=192.168.1.143
rem SET IS_BROADCAST=192.168.111.142
SET IS_BROADCAST=192.168.111.11

REM Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
REM e.g., UDPSIMMODE=0 equals to UDPSIMMODE=UDP_Simple
SET UDPSIMMODE=2

REM Set the path of the RflySim tools
SET PSP_PATH=C:\PX4PSP
SET PSP_PATH_LINUX=/mnt/c/PX4PSP
C:

:Top
ECHO.
ECHO ---------------------------------------
REM Max vehicle number 50
SET /a MAX_VEHICLE=50
REM SET /P VehicleNum=Please input UAV swarm number:
SET /A VehicleNum=1
SET /A Evaluated=VehicleNum
if %Evaluated% EQU %VehicleNum% (
    IF %VehicleNum% GTR 0 (
        IF %VehicleNum% GTR %MAX_VEHICLE% (
            ECHO The vehicle number is too large, which may cause a crash
            pause
        )
        GOTO StartSim
    )
    ECHO Not a positive integer
    GOTO Top
) ELSE (
    ECHO Not a integer
    GOTO Top
)
:StartSim

SET /A VehicleTotalNum=%VehicleNum% + %START_INDEX% - 1
if not defined TOTOAL_COPTER (
    SET /A TOTOAL_COPTER=%VehicleTotalNum%
)

set /a sqrtNum=1
set /a squareNum=1
:loopSqrt
set /a squareNum=%sqrtNum% * %sqrtNum%
if %squareNum% EQU %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
if %squareNum% GTR %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
set /a sqrtNum=%sqrtNum%+1
goto loopSqrt
:loopSqrtEnd


REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe
ECHO Start QGroundControl

REM UE4Path
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /f /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd %PSP_PATH%\CopterSim


set /a cntr=%START_INDEX%
set /a endNum=%VehicleTotalNum%+1
set /a portNum=%UDP_START_PORT%+((%START_INDEX%-1)*2)
:loopBegin
set /a PosXX=((%cntr%-1) / %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_X%
set /a PosYY=((%cntr%-1) %% %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_Y%
rem start CopterSim.exe 1 1 20100 0 2 3 192.168.1.143 0 0 180 1 2
start CopterSim.exe 1 %cntr% %portNum% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% 1 %UDPSIMMODE%
choice /t 1 /d y /n >nul
set /a cntr=%cntr%+1
set /a portNum=%portNum%+2
if %cntr% EQU %endNum% goto loopEnd
goto loopBegin
:loopEnd


REM client_grass
rem start cmd /c "%PSP_PATH%\Python38\python.exe %PSP_PATH%\RflySimAPIs\SLAM\vins_client.py %IS_BROADCAST% | tee %PSP_PATH%\RflySimAPIs\SLAM\pylog.txt"
REM start cmd /c "%PSP_PATH%\Python38\python.exe %PSP_PATH%\RflySimAPIs\SLAM\vins_client.py %IS_BROADCAST%"

REM Set ToolChainType 1:Win10WSL 3:Cygwin
SET /a ToolChainType=1

if "%IS_BROADCAST%" == "0" (
    SET IS_BROADCAST=0
) else (
    SET IS_BROADCAST=1
)


rem ./Tools/sitl_multiple_run_rfly.sh %VehicleNum% %START_INDEX% %IS_BROADCAST%
rem ./Tools/sitl_multiple_run_rfly.sh 1 1 1
if %ToolChainType% EQU 1 (
    wsl echo Starting PX4 Build; cd %PSP_PATH_LINUX%/Firmware; export PATH=$HOME/ninja:$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH;make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh %VehicleNum% %START_INDEX% %IS_BROADCAST%;echo Press any key to exit; read -n 1
) else (
    REM CYGPath
    cd %PSP_PATH%\CygwinToolchain
    CALL setPX4Env.bat
    start bash -l -i -c 'echo Starting SITL SIMULATION; cd %PSP_PATH_LINUX%/Firmware; pwd; make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh %VehicleNum% %START_INDEX% %IS_BROADCAST%;echo Press any key to exit; read -n 1;'
)


REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /f /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"
tasklist|find /i "python.exe" && taskkill /f /im "python.exe"

ECHO Start End.
