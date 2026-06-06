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

# ZLG CAN driver comes from vr-mc-sdk via the platform_linux target
# (linked further down). The diagnostic no longer carries its own fork
# of the ZLG sources — the SDK version supports multi-endpoint sharing
# on a single channel, which the in-tree fork did not, and which is
# required for the PDO + USDO server to co-exist on real ZLG hardware.
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
    src/MotorView.cpp
    src/MotorView.hpp
    src/SlavePickerBar.cpp
    src/SlavePickerBar.hpp
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
    src/BwFromKp.hpp
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

# ---- vrmc_sim — in-tree CiA 402 simulator for Help → Start demo ---------
#
# Independent of vr-mc-sdk's compiled cia402_drive_sim binary: the
# diagnostic ships its own simulator built from sources under tools/.
# Still links the SDK's slave-side libs (tinysh, can_stack, cia402,
# platform_linux) which the add_subdirectory above already exposes,
# but the resulting executable lives in the diagnostic build tree so
# operators don't need the SDK binary to run the demo.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(vrmc_sim
        tools/vrmc_sim.c
        tools/vrmc_sim_od.c
        tools/vrmc_sim_od.h
    )
    target_compile_definitions(vrmc_sim PRIVATE USE_LINUX_CAN=1)
    target_link_libraries(vrmc_sim PRIVATE
        tinysh
        can_stack
        cia402
        platform_linux
    )
    install(TARGETS vrmc_sim RUNTIME DESTINATION bin)
endif()
