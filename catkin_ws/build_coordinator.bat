@echo off
REM 编译多无人机协同追踪系统
REM 版本: v10.3.0-coordinator
REM 日期: 2025-10-17

echo ========================================
echo 多无人机协同追踪系统 - 编译脚本
echo ========================================
echo.

cd /d E:\robocup2025\catkin_ws

echo [1/4] 清理旧的编译文件...
if exist build\yolov11_ros_msgs (
    rmdir /s /q build\yolov11_ros_msgs
)
if exist devel\lib\python3\dist-packages\yolov11_ros_msgs (
    rmdir /s /q devel\lib\python3\dist-packages\yolov11_ros_msgs
)

echo.
echo [2/4] 编译消息定义...
call devel\setup.bat
catkin_make --pkg yolov11_ros_msgs

if %errorlevel% neq 0 (
    echo.
    echo ❌ 消息编译失败！
    pause
    exit /b 1
)

echo.
echo [3/4] 设置脚本执行权限...
cd src\yolov11_ros\scripts
if exist target_coordinator.py (
    echo    ✓ target_coordinator.py
) else (
    echo    ✗ target_coordinator.py 不存在！
    pause
    exit /b 1
)

echo.
echo [4/4] 验证消息定义...
call ..\..\..\devel\setup.bat
rosmsg show TargetRequest 2>nul
if %errorlevel% neq 0 (
    echo    ✗ TargetRequest 消息未找到
) else (
    echo    ✓ TargetRequest
)

rosmsg show TargetResponse 2>nul
if %errorlevel% neq 0 (
    echo    ✗ TargetResponse 消息未找到
) else (
    echo    ✓ TargetResponse
)

rosmsg show TargetHeartbeat 2>nul
if %errorlevel% neq 0 (
    echo    ✗ TargetHeartbeat 消息未找到
) else (
    echo    ✓ TargetHeartbeat
)

rosmsg show CoordinatorStatus 2>nul
if %errorlevel% neq 0 (
    echo    ✗ CoordinatorStatus 消息未找到
) else (
    echo    ✓ CoordinatorStatus
)

echo.
echo ========================================
echo ✅ 编译完成！
echo ========================================
echo.
echo 下一步：
echo 1. 启动PX4仿真: cd E:\robocup2025\PX4_Firmware ^& make px4_sitl_default gazebo
echo 2. 启动系统: roslaunch yolov11_ros multi_drone_system.launch
echo.
pause
