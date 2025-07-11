@echo off
echo G1机器人仿真启动脚本
echo.
echo 请选择控制方式:
echo 1. 手柄控制 (默认)
echo 2. 键盘控制
echo 3. 测试键盘控制器
echo 4. 安装依赖包
echo.
set /p choice="请输入选择 (1-4): "

if "%choice%"=="1" (
    echo 启动手柄控制模式...
    python deploy_mujoco\deploy_mujoco.py
) else if "%choice%"=="2" (
    echo 启动键盘控制模式...
    python deploy_mujoco\deploy_mujoco_keyboard.py --controller keyboard
) else if "%choice%"=="3" (
    echo 启动键盘控制器测试...
    python test_keyboard.py
) else if "%choice%"=="4" (
    echo 安装Python依赖包...
    pip install -r requirements.txt
    echo 依赖包安装完成
    pause
) else (
    echo 默认启动手柄控制模式...
    python deploy_mujoco\deploy_mujoco.py
)

pause
