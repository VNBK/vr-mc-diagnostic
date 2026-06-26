/*
 * boot_selftest.c - reproduce the diagnostic's firmware-upgrade transport
 * exactly, to verify the pumpForBoot() fix.
 *
 * The diagnostic (CanBackend) keeps TWO hal_can_udp endpoints on the same
 * multicast group: m_canCli (the normal USDO client) and m_canPdo (where the
 * boot USDO client is created during an upgrade). This mirrors that:
 *
 *   --interfere : poll BOTH endpoints and run the normal client too — the
 *                 old MasterWorker::onBootTick (m_can.pump()) behaviour.
 *   (default)   : poll ONLY the boot endpoint — the new pumpForBoot().
 *
 * Run vrmc_boot_sim as the slave, then this against it. If --interfere
 * fails where the default succeeds, the shared-bus interference is real and
 * pumpForBoot() is the fix.
 */
#include "hal_can_udp.h"
#include "co_fd_usdo.h"
#include "boot_master.h"
#include "boot_slave.h"
#include "interface/boot_if_impl.h"
#include "platform_porting.h"
#include "logger.h"

#include <stdio.h>
#include <string.h>

static volatile int g_done = 0, g_ok = 0;

static void on_ev(uint8_t ev, int32_t det, void* arg){
    (void)arg;
    switch(ev){
    case BOOT_EVENT_UPGRADE_STARTING:      printf("  STARTING\n"); break;
    case BOOT_EVENT_UPGRADE_SYSTEM_READY:  printf("  READY (5%%)\n"); break;
    case BOOT_EVENT_UPGRADE_IN_PROCESSING: printf("  segment %d\n", (int)det); break;
    case BOOT_EVENT_UPGRADE_FINISH:
        g_done = 1; g_ok = (det == BOOT_UPGRADING_ERR_NONE);
        printf("  FINISH err=%d\n", (int)det);
        break;
    default: break;
    }
}

int main(int argc, char** argv){
    int interfere = 0; const char* file = "/tmp/fw_4096.bin";
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--interfere")) interfere = 1;
        else file = argv[i];
    }
    platform_porting_cfg_t pc = platform_porting_default_cfg_linux();
    pc.can = NULL; pc.shell_rx = NULL; pc.initial_log_level = LOG_LEVEL_WARN;
    platform_porting_init(&pc);

    printf("mode = %s, file = %s\n", interfere ? "INTERFERE (old pump)" :
                                                 "boot-only (pumpForBoot)", file);

    hal_can_t* cli = hal_can_udp_create(NULL, 0);   /* normal endpoint */
    hal_can_t* pdo = hal_can_udp_create(NULL, 0);   /* boot endpoint   */
    co_fd_usdo_client_t* normal = co_fd_usdo_client_create(cli, NULL);

    boot_master_t* m  = boot_master_create();
    boot_input_t*  in = boot_input_file_create(file);
    boot_output_t* out= boot_output_cofd_create(pdo, 10);
    boot_slave_t*  sl = boot_slave_create(10, in, out, m);
    boot_master_add_slave(m, sl);
    boot_master_start_upgrade(m, on_ev, NULL);

    for(int i=0;i<60000 && !g_done;i++){
        if(interfere){
            hal_can_udp_poll(cli);
            hal_can_udp_poll(pdo);
            co_fd_usdo_client_process(normal, 1);   /* the old shared-bus path */
        } else {
            hal_can_udp_poll(pdo);                  /* pumpForBoot: boot only  */
        }
        boot_master_process(m);
        platform_sleep_ms(1);
    }
    printf("RESULT: %s\n", g_done ? (g_ok ? "OK" : "FAIL") : "TIMEOUT");
    boot_master_destroy(m);
    return g_ok ? 0 : 1;
}
