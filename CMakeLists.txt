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
find_package(Qt6 6.2 REQUIRED COMPONENTS Core Gui Widgets Charts PrintSupport)

# ---- yaml-cpp -----------------------------------------------------------
# Schema-v2 test database persistence, mirroring the vr_hand_diagnostic
# TestRunner. yaml-cpp is shipped as a system package
# (libyaml-cpp-dev on Debian/Ubuntu).
find_package(yaml-cpp REQUIRED)

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

# Feetech / Dynamixel RS-485 servo backends were removed — this
# diagnostic targets CANopen / CiA-402 motor drives (UDP loopback or
# ZLG USB-CANFD only).
set(BUILD_FEETECH   OFF CACHE BOOL "" FORCE)
set(BUILD_DYNAMIXEL OFF CACHE BOOL "" FORCE)

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
    src/FaultDiagDialog.cpp
    src/FaultDiagDialog.hpp
    src/ObjectDictionaryDialog.cpp
    src/ObjectDictionaryDialog.hpp
    src/RunInDialog.cpp
    src/RunInDialog.hpp
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
    src/TestRunner/TestRunnerWindow.cpp
    src/TestRunner/TestRunnerWindow.hpp
    src/TestRunner/YamlBridge.cpp
    src/TestRunner/YamlBridge.hpp
    src/TestRunner/AppConfig.hpp
    src/TestRunner/HandWorkerCompat.cpp
    src/TestRunner/HandWorkerCompat.hpp
    src/TestRunner/AddTestDialog.cpp
    src/TestRunner/AddTestDialog.hpp
    src/TestRunner/DatabaseEditDialog.cpp
    src/TestRunner/DatabaseEditDialog.hpp
    src/TestRunner/ParamsEditDialog.cpp
    src/TestRunner/ParamsEditDialog.hpp
    src/TestRunner/TestDescriptionDialog.cpp
    src/TestRunner/TestDescriptionDialog.hpp
    src/TestRunner/TestResultDialog.cpp
    src/TestRunner/TestResultDialog.hpp
    # probes/ folder ships as on-disk reference templates only — each
    # one drives a HandWorker via setPosition / setForce / etc., so it
    # needs porting to MasterWorker before it can be added to the build.
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
    Qt6::PrintSupport
    master_mgr
    motor_drive_cia402_intf
    cia402_master
    can_stack
    platform_linux
    vr_bootmaster          # boot_master + boot_slave + file-input + cofd-output
    yaml-cpp               # TestRunner schema-v2 IO
)


set_target_properties(vr_mc_diagnostic PROPERTIES
    BUILD_RPATH   "${VR_MC_SDK_DIR}/src/platform/linux/zlg/lib"
    INSTALL_RPATH "$ORIGIN/zlg/lib")

install(TARGETS vr_mc_diagnostic RUNTIME DESTINATION bin)

# Ship the TestRunner config tree (motorControlTestDB.yaml + future
# per-hand files) alongside the binary so QStandardPaths::locate finds
# it at runtime under share/vr_mc_diagnostic/config/.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/"
            DESTINATION share/vr_mc_diagnostic/config
            FILES_MATCHING
                PATTERN "*.yaml"
                PATTERN "*.yml"
                PATTERN "*.json")
endif()

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

    # ---- vrmc_boot_sim — host bootloader slave for upgrade testing ------
    #
    # Counterpart to vrmc_sim: runs the SDK boot core (vr_bootcore) over a
    # UDP-multicast "CAN bus" (vr_bootport_linux) so the diagnostic's
    # Tools -> Firmware upgrade can be exercised end-to-end on one machine,
    # before touching real F2800154 hardware. The "flashed" image lands as
    # app.bin under --work (default /tmp/vrboot).
    if(TARGET vr_bootcore AND TARGET vr_bootport_linux)
        add_executable(vrmc_boot_sim tools/vrmc_boot_sim.c)
        target_link_libraries(vrmc_boot_sim PRIVATE
            vr_bootcore
            vr_bootport_linux
            platform_linux
        )
        install(TARGETS vrmc_boot_sim RUNTIME DESTINATION bin)
    endif()

    # boot_selftest — headless repro of the diagnostic's upgrade transport,
    # used to validate the pumpForBoot() shared-bus fix (not installed).
    if(TARGET vr_bootmaster)
        add_executable(boot_selftest tools/boot_selftest.c)
        target_link_libraries(boot_selftest PRIVATE
            vr_bootmaster
            can_stack
            platform_linux
        )
    endif()
endif()
