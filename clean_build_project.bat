@echo off
REM ====================================================
REM GigaLearn2 Build Script
REM Deletes build folder contents, runs CMake, builds project
REM ====================================================

REM set to your project build dir
SET BUILD_DIR=C:\Programming\CPP\GigaLearn2\build

echo Cleaning build directory: %BUILD_DIR%

REM Always delete everything including checkpoints
echo Deleting all contents of build directory (including checkpoints)...

REM Remove all files and folders in build dir
for /D %%d in ("%BUILD_DIR%\*") do (
    echo Deleting folder %%d
    rmdir /s /q "%%d" 2>nul
)

REM Remove any loose files in the build folder (not directories)
for %%f in ("%BUILD_DIR%\*") do (
    if not "%%~nxf"=="Release" (
        echo Deleting file %%f
        del /f /q "%%f" 2>nul
    )
)

REM Ensure build directory exists
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM Change into build directory
cd /d "%BUILD_DIR%"

REM Set paths and directories to your own
echo Running CMake configuration...
"C:\Program Files\CMake\bin\cmake.exe" .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_GENERATOR_TOOLSET="cuda=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8" ^
    -DCMAKE_PREFIX_PATH="C:/Programming/CPP/GigaLearn2/libtorch" ^
    -DPython_EXECUTABLE="C:\Program Files\Python314\python.exe" ^
    -DPython_INCLUDE_DIR="C:/Program Files/Python314/include" ^
    -DPython_LIBRARY="C:/Program Files/Python314/libs/python314.lib"

echo Building project...
"C:\Program Files\CMake\bin\cmake.exe" --build . --config Release

echo Build complete!