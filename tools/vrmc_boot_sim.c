/*
 * vrmc_boot_sim.c - host-side CANopen-FD bootloader slave for testing the
 *                   diagnostic's firmware-upgrade flow without hardware.
 *
 * Counterpart to vrmc_sim (which simulates the CiA-402 *drive*): this
 * binary simulates the *bootloader* a board comes up in after an
 * enter-boot. It runs the SDK boot core (vr_bootcore) over a UDP-multicast
 * "CAN bus" via boot_porting_linux, parking in WAITING_CMD and answering
 * the 0x3003 upgrade object-set. Point the diagnostic at it in UDP mode
 * and run Tools -> Firmware upgrade to exercise the whole path
 * (MasterWorker boot_master -> 0x3003 SDO transfer -> "flash" under the
 * work dir) end-to-end on one machine.
 *
 * This is a near-verbatim port of vr-mc-sdk/app/bootloader_node.c, kept in
 * the diagnostic tree so operators get the test target from the same build
 * as the GUI (same pattern as tools/vrmc_sim.c shipping its own drive sim).
 *
 * Typical loop-back test:
 *     # terminal 1 — the simulated board in bootloader mode
 *     ./build/vrmc_boot_sim --work /tmp/vrboot
 *
 *     # terminal 2 — the GUI, connected via the UDP (sim) transport,
 *     #              then Tools -> Firmware upgrade -> pick an image
 *     ./build/vr_mc_diagnostic
 *
 * Both bind to the hal_can_udp default multicast group so they share the
 * virtual bus. The "flashed" image lands as app.bin under --work.
 */

#include "bootloader.h"
#include "boot_porting_linux.h"

#include "platform_porting.h"
#include "logger.h"

#include <signal.h>
#include <stdlib.h>
#include <string.h>

#define TAG "vrmc-boot-sim"

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig){ (void)sig; g_stop = 1; }

static void print_help(const char* argv0)
{
    LOG_INFO(TAG, "Usage: %s [options]", argv0);
    LOG_INFO(TAG, "  --work <dir>    Storage dir for the 'flashed' image "
                  "(default /tmp/vrboot)");
    LOG_INFO(TAG, "  --group <addr>  UDP multicast group (default matches "
                  "the GUI's --udp)");
    LOG_INFO(TAG, "  --port <n>      UDP port (default matches the GUI)");
    LOG_INFO(TAG, "  --ticks <n>     Exit after N ticks (headless test)");
    LOG_INFO(TAG, "  -h, --help      Show this help");
}

int main(int argc, char** argv)
{
    /* Logger sink first so boot_porting_linux_init's lines are captured. */
    platform_porting_cfg_t pcfg = platform_porting_default_cfg_linux();
    pcfg.can               = NULL;       /* the boot port owns CAN */
    pcfg.shell_rx          = NULL;       /* no shell here          */
    pcfg.initial_log_level = LOG_LEVEL_INFO;
    platform_porting_init(&pcfg);

    const char* work_dir = NULL;
    const char* can_grp  = NULL;
    uint16_t    can_port = 0;
    int         max_ticks = 0;

    for(int i = 1; i < argc; i++){
        if(!strcmp(argv[i], "--work") && i + 1 < argc){
            work_dir = argv[++i];
        } else if(!strcmp(argv[i], "--group") && i + 1 < argc){
            can_grp  = argv[++i];
        } else if(!strcmp(argv[i], "--port") && i + 1 < argc){
            can_port = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if(!strcmp(argv[i], "--ticks") && i + 1 < argc){
            max_ticks = (int)strtoul(argv[++i], NULL, 0);
        } else if(!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")){
            print_help(argv[0]);
            return 0;
        }
    }

    if(boot_porting_linux_init(work_dir, can_grp, can_port) != 0){
        LOG_ERR(TAG, "boot_porting_linux_init FAILED");
        return 1;
    }

    boot_t* boot = boot_create();
    if(!boot){
        LOG_ERR(TAG, "boot_create FAILED — check the error lines above");
        boot_porting_linux_teardown();
        return 1;
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    LOG_INFO(TAG, "vrmc_boot_sim running (bootloader mode). Ctrl+C to exit.");
    LOG_INFO(TAG, "Connect the diagnostic via the UDP transport, then run "
                  "Tools -> Firmware upgrade to flash this node.");

    int ticks = 0;
    while(!g_stop){
        boot_porting_linux_pump_can();   /* drain UDP RX -> SDO server cb */
        boot_process(boot);
        if(boot_porting_linux_jump_requested()){
            LOG_INFO(TAG, "bootloader signalled jump — image accepted; "
                          "exiting sim (a real board would run the new app)");
            break;
        }
        if(max_ticks && ++ticks >= max_ticks){
            LOG_INFO(TAG, "reached --ticks=%d, exiting", max_ticks);
            break;
        }
        platform_sleep_ms(1);
    }

    LOG_INFO(TAG, "shutting down");
    boot_destroy(boot);
    boot_porting_linux_teardown();
    platform_porting_shutdown();
    return 0;
}
