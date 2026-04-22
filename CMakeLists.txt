cmake_minimum_required(VERSION 3.16)
project(vr_mc_diagnostic CXX C)

# ---- Compile flags ------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# Qt moc/uic/rcc auto-run on all headers + .qrc in add_executable.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---- Qt6 ----------------------------------------------------------------
find_package(Qt6 6.2 REQUIRED COMPONENTS Core Gui Widgets Charts)

# ---- vr-mc-sdk master libs (via add_subdirectory) -----------------------
# VR_MC_SDK_DIR defaults to the sibling checkout at ../vr-mc-sdk. Override
# with -DVR_MC_SDK_DIR=/abs/path when pulling an out-of-tree copy.
set(VR_MC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vr-mc-sdk"
    CACHE PATH "Path to vr-mc-sdk checkout")

if(NOT EXISTS "${VR_MC_SDK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "vr-mc-sdk not found at ${VR_MC_SDK_DIR} - set -DVR_MC_SDK_DIR=...")
endif()

# Diagnostic only needs libs — don't build SDK tests / examples / apps.
set(BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_APP      OFF CACHE BOOL "" FORCE)

# Feetech SMS/STS backend ships with the SDK (no external deps) — always
# on so the transport menu has it out of the box. Dynamixel uses an
# in-tree Protocol 2.0 driver (src/backends/DynamixelDriver.cpp) — no
# ROBOTIS SDK dependency, always available at runtime.
set(BUILD_FEETECH   ON  CACHE BOOL "" FORCE)
set(BUILD_DYNAMIXEL OFF CACHE BOOL "" FORCE)   # SDK adapter unused now

add_subdirectory(${VR_MC_SDK_DIR} ${CMAKE_BINARY_DIR}/_vr_mc_sdk EXCLUDE_FROM_ALL)

# ---- ZLG CAN driver (in-tree, similar shape to hal_can_udp) -------------
# Vendor sources copied into src/backends/zlg/ so the diagnostic doesn't
# depend on vr-hand-sdk or any external SDK. The native runtime
# (libcontrolcanfd.so) ships in src/backends/zlg/lib/ and is dlopen'd by
# ZlgCanDrv at startup; the executable RPATH points there so it Just Works.
add_library(zlg_can STATIC
    src/backends/zlg/zlg_can_drv.cpp
    src/backends/zlg/zlg_can_drv.hpp
    src/backends/zlg/can_drv.hpp
    src/backends/zlg/controlcanfd.h
    src/backends/zlg/config.h
    src/backends/zlg/timer.h
    src/backends/zlg/logger.hpp
    src/backends/zlg/hal_can_zlg.cpp
    src/backends/zlg/hal_can_zlg.hpp
)
target_include_directories(zlg_can PUBLIC
    src/backends/zlg
    ${VR_MC_SDK_DIR}/src/platform/hal
    ${VR_MC_SDK_DIR}/src/mc/libraries/utilities/types/include)
target_link_libraries(zlg_can PUBLIC dl pthread)
set_target_properties(zlg_can PROPERTIES
    BUILD_RPATH   "${CMAKE_CURRENT_SOURCE_DIR}/src/backends/zlg/lib"
    INSTALL_RPATH "$ORIGIN/zlg/lib")

# ---- Executable ---------------------------------------------------------
add_executable(vr_mc_diagnostic
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.hpp
    src/ConnectionDialog.cpp
    src/ConnectionDialog.hpp
    src/MasterWorker.cpp
    src/MasterWorker.hpp
    src/MotorProfile.cpp
    src/MotorProfile.hpp
    src/MotorProfileEditor.cpp
    src/MotorProfileEditor.hpp
    src/MotorProfileView.cpp
    src/MotorProfileView.hpp
    src/PdoMappingView.cpp
    src/PdoMappingView.hpp
    src/PdoMappingEditor.cpp
    src/PdoMappingEditor.hpp
    src/DriveConfigDialog.cpp
    src/DriveConfigDialog.hpp
    src/FirmwareUpgradeDialog.cpp
    src/FirmwareUpgradeDialog.hpp
    src/SlaveTableModel.cpp
    src/SlaveTableModel.hpp
    src/JointControlPanel.cpp
    src/JointControlPanel.hpp
    src/GainEditor.cpp
    src/GainEditor.hpp
    src/SignalGeneratorPanel.cpp
    src/SignalGeneratorPanel.hpp
    src/TuningPane.cpp
    src/TuningPane.hpp
    src/TelemetryWidget.cpp
    src/TelemetryWidget.hpp
    src/LogDock.cpp
    src/LogDock.hpp
    src/backends/CanBackend.cpp
    src/backends/CanBackend.hpp
    src/backends/DynamixelDriver.cpp
    src/backends/DynamixelDriver.hpp
    resources/vrmc.qrc
)

target_include_directories(vr_mc_diagnostic PRIVATE src)

# Link the C master stack. master_mgr + cia402 backend pulls in cia402,
# cia402_master, can_stack, platform_linux transitively via their own
# PUBLIC link interfaces.
target_link_libraries(vr_mc_diagnostic PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Charts
    zlg_can
    master_mgr
    motor_drive_cia402_intf
    cia402_master
    can_stack
    platform_linux
    motor_drive_feetech_intf
    feetech_servo
)


set_target_properties(vr_mc_diagnostic PROPERTIES
    BUILD_RPATH   "${CMAKE_CURRENT_SOURCE_DIR}/src/backends/zlg/lib"
    INSTALL_RPATH "$ORIGIN/zlg/lib")

install(TARGETS vr_mc_diagnostic RUNTIME DESTINATION bin)
