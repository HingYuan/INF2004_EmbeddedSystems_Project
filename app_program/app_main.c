/**
 * @file app_main.c
 * @brief System initialization, IPC demo pipeline, and Wi-Fi status monitor.
 *
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas (base demo harness)
 * SPDX-License-Identifier: Apache-2.0
 *
 * BARR-C Deviation (Rules 1.5.a, 2.1.c.ii, 5.2.a, 6.1.g, 6.1.h, 6.1.i):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Entry symbol INT usermain(void) is fixed by micro T-Kernel 3.0;
 *   RTOS syscalls and T-Monitor console APIs require native INT/UINT/UB/SZ/TMO
 *   types; T_CYW43_UTK_STATUS.udp_elapsed_ms is defined in legacy Wi-Fi driver
 *   header cyw43_utk.h; serial logs report live system/network status.
 */

#include "app_main.h"

#include <stdbool.h>
#include <stdint.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include <tm/tmonitor.h>

#include "demo_tasks.h"
#include "robot_tasks.h"
#include "usb_console_compat.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

#define APP_MAIN_ENABLE_DEMO_MON        (0u)
#define APP_MAIN_RECORD_COUNT           (4u)
#define APP_MAIN_CREDITS                (2u)
#define APP_MAIN_PRODUCE_MSEC           (500u)
#define APP_MAIN_REPORT_MSEC            (2000u)
#define APP_MAIN_STACK_BYTES            (4096u)
#define APP_MAIN_SEM_WAIT_MSEC          (1000u)
#define APP_MAIN_MPF_WAIT_MSEC          (100u)
#define APP_MAIN_MBF_WAIT_MSEC          (1000u)
#define APP_MAIN_PAYLOAD_MULT           (7u)
#define APP_MAIN_FLG_PRODUCED           (1u << 0u)
#define APP_MAIN_FLG_CONSUMED           (1u << 1u)
#define APP_MAIN_USB_WAIT_LOOPS         (200u)
#define APP_MAIN_USB_WAIT_STEP_MSEC     (100u)
#define APP_MAIN_USB_READY_STATE        (2u)
#define APP_MAIN_MON_SETTLE_MSEC        (500u)
#define APP_MAIN_PRODUCER_PRIO          (5)
#define APP_MAIN_CONSUMER_PRIO          (6)
#define APP_MAIN_MONITOR_PRIO           (7)
#define APP_MAIN_WIFI_PRIO              (8)
#define APP_MAIN_BLINK_PRIO             (10)

#if TM_WIFI_CYW43
#define APP_MAIN_WIFI_IDLE_MSEC         (3000u)
#define APP_MAIN_WIFI_BUSY_MSEC         (200u)
#define APP_MAIN_WIFI_SETTLE_MSEC       (700u)
#define APP_MAIN_WIFI_UNSET_VAL         (0xFFFFFFFFu)
#define APP_MAIN_UDP_STALL_POLLS        (25u)
#define APP_MAIN_UDP_PREVIEW_BYTES      (12u)
#define APP_MAIN_IPV4_BUF_LEN           (16u)
#define APP_MAIN_UDP_LINE_BUF_LEN       (96u)
#endif

#define APP_MAIN_MPF_WORDS \
    ((((APP_MAIN_RECORD_COUNT * sizeof(app_main_record_t)) + \
       sizeof(uint32_t) - 1u) / sizeof(uint32_t)) + APP_MAIN_RECORD_COUNT)
#define APP_MAIN_MBF_WORDS \
    (((APP_MAIN_CREDITS * sizeof(app_main_record_t)) / sizeof(uint32_t)) + \
     (APP_MAIN_CREDITS * 4u))

typedef struct
{
    uint32_t seq_num;
    int32_t  made_on;
    uint32_t payload;
} app_main_record_t;

#if TM_WIFI_CYW43
typedef struct
{
    uint32_t last_link;
    uint32_t last_dhcp;
    uint32_t echo_cursor;
    uint32_t stalled_polls;
    uint32_t shown_session;
    uint32_t summarised_session;
} app_main_wifi_track_t;
#endif

static ID                gh_mpf_id       = 0;
static ID                gh_mtx_id       = 0;
static ID                gh_mbf_id       = 0;
static ID                gh_sem_id       = 0;
static ID                gh_flg_id       = 0;
static volatile uint32_t g_next_seq      = 0u;
static volatile uint32_t g_produced_cnt  = 0u;
static volatile uint32_t g_consumed_cnt  = 0u;
static volatile uint32_t g_dropped_cnt   = 0u;
static volatile int32_t  g_last_made_on  = 0;
static volatile int32_t  g_last_seen_on  = 0;
static uint32_t          g_mpf_buf[APP_MAIN_MPF_WORDS] = { 0u };
static uint32_t          g_mbf_buf[APP_MAIN_MBF_WORDS] = { 0u };

static bool app_main_create_ipc_objects(void);
static void app_main_launch_task(const char * p_label,
                                 FP p_entry,
                                 INT priority,
                                 uint32_t assprc);
static void app_main_spawn_demo_tasks(void);
static void app_main_record_drop(void);
static void app_main_produce_one(void);
static void app_main_producer_task(INT stacd, void * p_exinf);
static void app_main_consumer_task(INT stacd, void * p_exinf);
static bool app_main_check_created(const char * p_what, ID h_obj);
static void app_main_wait_usb_enum(uint32_t settle_msec);
static int32_t app_main_this_prc(void);

#if (0u != APP_MAIN_ENABLE_DEMO_MON)
static void app_main_monitor_task(INT stacd, void * p_exinf);
#endif

#if TM_WIFI_CYW43
static void app_main_wifi_status_task(INT stacd, void * p_exinf);
static void app_main_fmt_ipv4(uint8_t * p_dst, const uint8_t * p_addr);
static void app_main_log_wifi_link(const T_CYW43_UTK_STATUS * p_status,
                                   app_main_wifi_track_t * p_track);
static void app_main_log_udp_echoes(const T_CYW43_UTK_STATUS * p_status,
                                    app_main_wifi_track_t * p_track);
static void app_main_log_udp_state(const T_CYW43_UTK_STATUS * p_status,
                                   app_main_wifi_track_t * p_track);
#endif

/**
 * @brief Application entry point invoked by the micro T-Kernel init task.
 * @return Zero on normal exit (never returns), or 1 on startup failure.
 */
INT usermain (void)
{
    INT  ret_code   = 0;
    bool b_is_ready = app_main_create_ipc_objects();

    if (b_is_ready)
    {
        app_main_spawn_demo_tasks();

        if (robot_tasks_start() < E_OK)
        {
            ret_code = 1;
        }
        else
        {
            /* Range-safe: initial task sleeps forever once system is live. */
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    else
    {
        ret_code = 1;
    }

    return ret_code;
}

/**
 * @brief Create fixed memory pool, mutex, message buffer, semaphore, and flag.
 * @return True if all five RTOS IPC objects were created, false otherwise.
 */
static bool app_main_create_ipc_objects (void)
{
    const T_CMPF cmpf_cfg =
    {
        .mpfatr = TA_TFIFO | TA_RNG3,
        /* Range-safe: pool count (4) fits positive 32-bit INT. */
        .mpfcnt = (INT)APP_MAIN_RECORD_COUNT,
        /* Range-safe: 12-byte record size fits positive 32-bit SZ. */
        .blfsz  = (SZ)sizeof(app_main_record_t),
        .bufptr = g_mpf_buf
    };
    const T_CMTX cmtx_cfg =
    {
        .mtxatr = TA_TFIFO | TA_INHERIT
    };
    const T_CMBF cmbf_cfg =
    {
        .mbfatr = TA_TFIFO,
        /* Range-safe: static message buffer byte size fits positive SZ. */
        .bufsz  = (SZ)sizeof(g_mbf_buf),
        /* Range-safe: 12-byte record size fits positive 32-bit INT. */
        .maxmsz = (INT)sizeof(app_main_record_t),
        .bufptr = g_mbf_buf
    };
    const T_CSEM csem_cfg =
    {
        .sematr  = TA_TFIFO,
        /* Range-safe: APP_MAIN_CREDITS (2u) fits positive 32-bit INT. */
        .isemcnt = (INT)APP_MAIN_CREDITS,
        /* Range-safe: APP_MAIN_CREDITS (2u) fits positive 32-bit INT. */
        .maxsem  = (INT)APP_MAIN_CREDITS
    };
    const T_CFLG cflg_cfg =
    {
        .flgatr  = TA_TFIFO | TA_WMUL,
        .iflgptn = 0u
    };
    bool         b_is_ready = false;

    gh_mpf_id  = tk_cre_mpf(&cmpf_cfg);
    b_is_ready = app_main_check_created("mpf", gh_mpf_id);

    if (b_is_ready)
    {
        gh_mtx_id  = tk_cre_mtx(&cmtx_cfg);
        b_is_ready = app_main_check_created("mtx", gh_mtx_id);
    }

    if (b_is_ready)
    {
        gh_mbf_id  = tk_cre_mbf(&cmbf_cfg);
        b_is_ready = app_main_check_created("mbf", gh_mbf_id);
    }

    if (b_is_ready)
    {
        gh_sem_id  = tk_cre_sem(&csem_cfg);
        b_is_ready = app_main_check_created("sem", gh_sem_id);
    }

    if (b_is_ready)
    {
        gh_flg_id  = tk_cre_flg(&cflg_cfg);
        b_is_ready = app_main_check_created("flg", gh_flg_id);
    }

    return b_is_ready;
}

/**
 * @brief Create and start one background uT-Kernel demo or status task.
 * @param[in] p_label  Diagnostic label printed if creation fails.
 * @param[in] p_entry  Task entry function pointer.
 * @param[in] priority Initial RTOS priority.
 * @param[in] assprc   Processor affinity mask (0u for unassigned).
 */
static void app_main_launch_task (const char * p_label,
                                  FP p_entry,
                                  INT priority,
                                  uint32_t assprc)
{
    T_CTSK ctsk   = { 0 };
    ID     h_task = 0;

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.itskpri = priority;
    /* Range-safe: 4096-byte stack size constant fits positive 32-bit SZ. */
    ctsk.stksz   = (SZ)APP_MAIN_STACK_BYTES;
    ctsk.task    = p_entry;

#if TK_SUPPORT_SMP
    if (0u != assprc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        /* Range-safe: processor affinity bitmask maps 1-to-1 to UW. */
        ctsk.assprc  = (UW)assprc;
    }
#else
    /* Range-safe: affinity unused on single-core builds. */
    (void)assprc;
#endif

    h_task = tk_cre_tsk(&ctsk);

    if (app_main_check_created(p_label, h_task))
    {
        /* Range-safe: valid h_task > 0 checked immediately above. */
        (void)tk_sta_tsk(h_task, 0);
    }
}

/**
 * @brief Create and start the background IPC demo and optional Wi-Fi tasks.
 */
static void app_main_spawn_demo_tasks (void)
{
    uint32_t demo_prc = 0u;

#if TK_SUPPORT_SMP
    demo_prc = TP_PRC1;
#endif

#if (0u != APP_MAIN_ENABLE_DEMO_MON)
    /* Range-safe: task function pointer matches uT-Kernel FP signature. */
    app_main_launch_task("tsk(blink)",
                         (FP)demo_tasks_blink_task,
                         APP_MAIN_BLINK_PRIO,
                         0u);
    /* Range-safe: task function pointer matches uT-Kernel FP signature. */
    app_main_launch_task("tsk(monitor)",
                         (FP)app_main_monitor_task,
                         APP_MAIN_MONITOR_PRIO,
                         0u);
#endif

    /* Range-safe: task function pointer matches uT-Kernel FP signature. */
    app_main_launch_task("tsk(consumer)",
                         (FP)app_main_consumer_task,
                         APP_MAIN_CONSUMER_PRIO,
                         demo_prc);
    /* Range-safe: task function pointer matches uT-Kernel FP signature. */
    app_main_launch_task("tsk(producer)",
                         (FP)app_main_producer_task,
                         APP_MAIN_PRODUCER_PRIO,
                         demo_prc);

#if TM_WIFI_CYW43
    {
        uint32_t wifi_prc = 0u;

#if TK_SUPPORT_SMP
        wifi_prc = TP_PRC2;
#endif
        /* Range-safe: task function pointer matches uT-Kernel FP signature. */
        app_main_launch_task("tsk(wifi)",
                             (FP)app_main_wifi_status_task,
                             APP_MAIN_WIFI_PRIO,
                             wifi_prc);
    }
#endif
}

/**
 * @brief Atomically increment the shared IPC dropped-record counter.
 */
static void app_main_record_drop (void)
{
    UINT saved_state = robot_tasks_data_lock();

    g_dropped_cnt++;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Return the 1-based processor ID of the calling task.
 * @return Current processor index (1 on single-core builds).
 */
static int32_t app_main_this_prc (void)
{
    int32_t prc_id = 1;

#if TK_SUPPORT_SMP
    /* Range-safe: RP2040 processor IDs are 1 or 2. */
    prc_id = (int32_t)tk_get_prc();
#endif

    return prc_id;
}

/**
 * @brief Verify that a micro T-Kernel object creation call succeeded.
 * @param[in] p_what Human-readable object label for diagnostics.
 * @param[in] h_obj  Returned object ID or negative error code.
 * @return True if creation succeeded, false otherwise.
 */
static bool app_main_check_created (const char * p_what, ID h_obj)
{
    bool b_is_ok = true;

    if (h_obj <= E_OK)
    {
        /* Range-safe: ASCII literal bytes map identically to const UB *. */
        tm_printf((const UB *)"[init] tk_cre_%s failed, er=%d\n",
                  p_what,
                  h_obj);
        b_is_ok = false;
    }

    return b_is_ok;
}

/**
 * @brief Wait for USB-CDC enumeration up to ~20 s, then settle briefly.
 * @param[in] settle_msec Additional post-enumeration delay in milliseconds.
 */
static void app_main_wait_usb_enum (uint32_t settle_msec)
{
    for (uint32_t wait_idx = 0u;
         (wait_idx < APP_MAIN_USB_WAIT_LOOPS) &&
         (usb_console_compat_state() < APP_MAIN_USB_READY_STATE);
         wait_idx++)
    {
        /* Range-safe: bounded task delay return status is non-critical. */
        (void)tk_dly_tsk(APP_MAIN_USB_WAIT_STEP_MSEC);
    }

    /* Range-safe: bounded post-enumeration settle delay. */
    (void)tk_dly_tsk(settle_msec);
}

/**
 * @brief Allocate, populate, and send one sequenced demo record.
 */
static void app_main_produce_one (void)
{
    void * p_blk   = NULL;
    /* Range-safe: 100 ms timeout constant fits positive signed 32-bit TMO. */
    ER     ret_err = tk_get_mpf(gh_mpf_id,
                                &p_blk,
                                (TMO)APP_MAIN_MPF_WAIT_MSEC);

    if (ret_err < E_OK)
    {
        app_main_record_drop();
        /* Range-safe: return unused credit to bounded counting semaphore. */
        (void)tk_sig_sem(gh_sem_id, 1);
    }
    else
    {
        /* Range-safe: p_blk comes from g_mpf_buf sized to app_main_record_t. */
        app_main_record_t * p_rec = (app_main_record_t *)p_blk;
        app_main_record_t   msg   = { 0u, 0, 0u };

        /* Range-safe: infinite wait on initialized mutex handle gh_mtx_id. */
        (void)tk_loc_mtx(gh_mtx_id, TMO_FEVR);
        g_next_seq++;
        p_rec->seq_num = g_next_seq;
        /* Range-safe: unlocks mutex held by the current task. */
        (void)tk_unl_mtx(gh_mtx_id);

        p_rec->made_on = app_main_this_prc();
        p_rec->payload = p_rec->seq_num * APP_MAIN_PAYLOAD_MULT;

        msg = *p_rec;
        /* Range-safe: releases block allocated from gh_mpf_id above. */
        (void)tk_rel_mpf(gh_mpf_id, p_blk);

        /* Range-safe: 12-byte msg size and 1000 ms timeout fit INT and TMO. */
        ret_err = tk_snd_mbf(gh_mbf_id,
                             &msg,
                             (INT)sizeof(msg),
                             (TMO)APP_MAIN_MBF_WAIT_MSEC);

        if (ret_err < E_OK)
        {
            app_main_record_drop();
            /* Range-safe: return unused credit on send timeout. */
            (void)tk_sig_sem(gh_sem_id, 1);
        }
        else
        {
            UINT saved_state = robot_tasks_data_lock();

            g_produced_cnt++;
            g_last_made_on = msg.made_on;
            robot_tasks_data_unlock(saved_state);

            /* Range-safe: notify optional monitor task and pace producer. */
            (void)tk_set_flg(gh_flg_id, APP_MAIN_FLG_PRODUCED);
            /* Range-safe: 500 ms delay return code is non-critical. */
            (void)tk_dly_tsk(APP_MAIN_PRODUCE_MSEC);
        }
    }
}

/**
 * @brief Producer task generating sequenced records over IPC objects.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
static void app_main_producer_task (INT stacd, void * p_exinf)
{
    /* Range-safe: intentionally unused RTOS task entry parameters. */
    (void)stacd;
    /* Range-safe: intentionally unused RTOS extended info pointer. */
    (void)p_exinf;

    for (;;)
    {
        /* Range-safe: 1000 ms wait constant fits positive signed TMO. */
        ER ret_err = tk_wai_sem(gh_sem_id, 1, (TMO)APP_MAIN_SEM_WAIT_MSEC);

        if (ret_err < E_OK)
        {
            app_main_record_drop();
        }
        else
        {
            app_main_produce_one();
        }
    }
}

/**
 * @brief Consumer task validating sequenced records and returning credits.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
static void app_main_consumer_task (INT stacd, void * p_exinf)
{
    /* Range-safe: intentionally unused RTOS task entry parameters. */
    (void)stacd;
    /* Range-safe: intentionally unused RTOS extended info pointer. */
    (void)p_exinf;

    for (;;)
    {
        app_main_record_t msg      = { 0u, 0, 0u };
        INT               msg_size = tk_rcv_mbf(gh_mbf_id, &msg, TMO_FEVR);
        /* Range-safe: 12-byte record size fits positive 32-bit INT. */
        INT               exp_size = (INT)sizeof(msg);

        if (exp_size != msg_size)
        {
            app_main_record_drop();
        }
        else if ((msg.seq_num * APP_MAIN_PAYLOAD_MULT) != msg.payload)
        {
            app_main_record_drop();
        }
        else
        {
            int32_t seen_prc    = app_main_this_prc();
            UINT    saved_state = robot_tasks_data_lock();

            g_consumed_cnt++;
            g_last_seen_on = seen_prc;
            robot_tasks_data_unlock(saved_state);

            /* Range-safe: signal consumed event and return semaphore credit. */
            (void)tk_set_flg(gh_flg_id, APP_MAIN_FLG_CONSUMED);
            /* Range-safe: bounded semaphore credit return. */
            (void)tk_sig_sem(gh_sem_id, 1);
        }
    }
}

#if (0u != APP_MAIN_ENABLE_DEMO_MON)
/**
 * @brief Optional demo monitor task printing IPC pipeline progress.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
static void app_main_monitor_task (INT stacd, void * p_exinf)
{
    /* Range-safe: intentionally unused RTOS task entry parameters. */
    (void)stacd;
    /* Range-safe: intentionally unused RTOS extended info pointer. */
    (void)p_exinf;

    app_main_wait_usb_enum(APP_MAIN_MON_SETTLE_MSEC);

    /* Range-safe: ASCII literal bytes map identically to const UB *. */
    tm_printf((const UB *)"\n=== uT-Kernel 3.0 / RP2040 demo ===\n");
#if TK_SUPPORT_SMP
    /* Range-safe: ASCII literal bytes map identically to const UB *. */
    tm_printf((const UB *)"SMP build: %d processors\n", TK_MAX_CORE);
#else
    /* Range-safe: ASCII literal bytes map identically to const UB *. */
    tm_printf((const UB *)"single-core build\n");
#endif
    /* Range-safe: ASCII literal bytes map identically to const UB *. */
    tm_printf((const UB *)"producer -> [mbf] -> consumer, %u credits, %u ms\n\n",
              APP_MAIN_CREDITS,
              APP_MAIN_PRODUCE_MSEC);

    for (;;)
    {
        UINT flg_ptn = 0u;
        /* Range-safe: 2000 ms timeout constant fits positive signed TMO. */
        ER   ret_err = tk_wai_flg(gh_flg_id,
                                  APP_MAIN_FLG_PRODUCED | APP_MAIN_FLG_CONSUMED,
                                  TWF_ANDW | TWF_CLR,
                                  &flg_ptn,
                                  (TMO)APP_MAIN_REPORT_MSEC);

        if (E_TMOUT == ret_err)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[monitor] stalled: produced=%u consumed=%u\n",
                      g_produced_cnt,
                      g_consumed_cnt);
        }
        else
        {
            uint32_t dropped_bytes = usb_console_compat_dropped();

            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[monitor] seq=%u prod=%u cons=%u drop=%u "
                      "made_on=prc%d seen_on=prc%d\n",
                      g_next_seq,
                      g_produced_cnt,
                      g_consumed_cnt,
                      g_dropped_cnt,
                      g_last_made_on,
                      g_last_seen_on);

            if (0u != dropped_bytes)
            {
                /* Range-safe: ASCII literal maps identically to const UB *. */
                tm_printf((const UB *)"[monitor] console dropped %u bytes\n",
                          dropped_bytes);
            }

            /* Range-safe: periodic monitor pacing delay. */
            (void)tk_dly_tsk(APP_MAIN_REPORT_MSEC);
        }
    }
}
#endif /* APP_MAIN_ENABLE_DEMO_MON */

#if TM_WIFI_CYW43
/**
 * @brief Format a 4-byte IPv4 address as a dotted-decimal string.
 * @param[out] p_dst  Destination buffer (at least 16 bytes).
 * @param[in]  p_addr Four-byte IPv4 address octets.
 */
static void app_main_fmt_ipv4 (uint8_t * p_dst, const uint8_t * p_addr)
{
    /* Range-safe: uint8_t buffer/octets map directly to UB * and UINT. */
    (void)tm_sprintf((UB *)p_dst,
                     (const UB *)"%u.%u.%u.%u",
                     (UINT)p_addr[0],
                     (UINT)p_addr[1],
                     (UINT)p_addr[2],
                     (UINT)p_addr[3]);
}

/**
 * @brief Report Wi-Fi link and DHCP state transitions to the serial console.
 * @param[in]     p_status Latest seqlock status snapshot from CYW43 service.
 * @param[in,out] p_track  Persistent transition tracker.
 */
static void app_main_log_wifi_link (const T_CYW43_UTK_STATUS * p_status,
                                    app_main_wifi_track_t * p_track)
{
    if (p_status->link_up != p_track->last_link)
    {
        p_track->last_link = p_status->link_up;

        if (0u != p_status->link_up)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[wifi] link UP  rssi=%d  "
                      "mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                      p_status->link_rssi,
                      p_status->mac[0],
                      p_status->mac[1],
                      p_status->mac[2],
                      p_status->mac[3],
                      p_status->mac[4],
                      p_status->mac[5]);
        }
        else
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[wifi] link DOWN\n");
        }
    }

    if (p_status->dhcp_complete != p_track->last_dhcp)
    {
        p_track->last_dhcp = p_status->dhcp_complete;

        if (0u != p_status->dhcp_complete)
        {
            uint8_t ip_str[APP_MAIN_IPV4_BUF_LEN]  = { 0u };
            uint8_t gw_str[APP_MAIN_IPV4_BUF_LEN]  = { 0u };
            uint8_t dns_str[APP_MAIN_IPV4_BUF_LEN] = { 0u };

            app_main_fmt_ipv4(ip_str, p_status->dhcp_address);
            app_main_fmt_ipv4(gw_str, p_status->dhcp_gateway);
            app_main_fmt_ipv4(dns_str, p_status->dhcp_dns);
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[wifi] ip=%s  gw=%s  dns=%s  "
                      "netif=%u/%u owner=prc%u\n",
                      ip_str,
                      gw_str,
                      dns_str,
                      p_status->netif_up,
                      p_status->netif_link_up,
                      p_status->owner_prc);
        }
    }
}

/**
 * @brief Drain newly arrived UDP echo entries from the status ring buffer.
 * @param[in]     p_status Latest seqlock status snapshot from CYW43 service.
 * @param[in,out] p_track  Persistent echo ring cursor tracker.
 */
static void app_main_log_udp_echoes (const T_CYW43_UTK_STATUS * p_status,
                                     app_main_wifi_track_t * p_track)
{
    /* Range-safe: positive ring capacity constant fits in uint32_t. */
    const uint32_t ring_cap = (uint32_t)CYW43_UDP_RING;

    if (p_status->udp_write_seq > p_track->echo_cursor)
    {
        if ((p_status->udp_write_seq - p_track->echo_cursor) > ring_cap)
        {
            uint32_t skipped =
                (p_status->udp_write_seq - p_track->echo_cursor) - ring_cap;

            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[udp] (%u echoes not shown, reader behind)\n",
                      skipped);
            p_track->echo_cursor = p_status->udp_write_seq - ring_cap;
        }

        while (p_track->echo_cursor < p_status->udp_write_seq)
        {
            uint32_t slot                                 =
                p_track->echo_cursor % ring_cap;
            uint8_t  line_buf[APP_MAIN_UDP_LINE_BUF_LEN] = { 0u };
            INT      pos                                  = 0;

            /* Range-safe: uint8_t buffer and ASCII format map to UB *. */
            pos = tm_sprintf((UB *)line_buf,
                             (const UB *)"[udp] echo #%u, %u bytes:",
                             p_status->udp_ring_seq[slot],
                             p_status->udp_ring_len[slot]);

            for (uint32_t byte_idx = 0u;
                 (byte_idx < APP_MAIN_UDP_PREVIEW_BYTES) &&
                 (byte_idx < p_status->udp_ring_len[slot]);
                 byte_idx++)
            {
                /* Range-safe: pos <= 88 stays within 96-byte line_buf. */
                pos += tm_sprintf((UB *)&line_buf[pos],
                                  (const UB *)" %02x",
                                  p_status->udp_ring_payload[slot][byte_idx]);
            }

            if (p_status->udp_ring_len[slot] > APP_MAIN_UDP_PREVIEW_BYTES)
            {
                /* Range-safe: 4-byte suffix fits within remaining buffer. */
                pos += tm_sprintf((UB *)&line_buf[pos], (const UB *)" ...");
            }

            /* Range-safe: final formatted length pos is not needed further. */
            (void)pos;
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"%s\n", line_buf);
            p_track->echo_cursor++;
        }
    }
}

/**
 * @brief Report UDP session start, stall, echoes, and completion summary.
 * @param[in]     p_status Latest seqlock status snapshot from CYW43 service.
 * @param[in,out] p_track  Persistent UDP progress tracker.
 */
static void app_main_log_udp_state (const T_CYW43_UTK_STATUS * p_status,
                                    app_main_wifi_track_t * p_track)
{
    uint8_t ip_str[APP_MAIN_IPV4_BUF_LEN] = { 0u };

    if ((0u == p_status->udp_packets_sent) &&
        (0u == p_status->udp_write_seq))
    {
        p_track->stalled_polls++;

        if (APP_MAIN_UDP_STALL_POLLS == p_track->stalled_polls)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[udp] waiting: link=%u netif_up=%u "
                      "link_up=%u dhcp=%u addr=%u.%u.%u.%u\n",
                      p_status->link_up,
                      p_status->netif_up,
                      p_status->netif_link_up,
                      p_status->dhcp_complete,
                      p_status->dhcp_address[0],
                      p_status->dhcp_address[1],
                      p_status->dhcp_address[2],
                      p_status->dhcp_address[3]);
            p_track->stalled_polls = 0u;
        }
    }
    else
    {
        p_track->stalled_polls = 0u;
    }

    if ((0u != p_status->udp_packets_sent) &&
        (p_status->udp_session_count != p_track->shown_session))
    {
        p_track->shown_session = p_status->udp_session_count;
        app_main_fmt_ipv4(ip_str, p_status->udp_target);
        /* Range-safe: ASCII literal bytes map identically to const UB *. */
        tm_printf((const UB *)"[udp] session %u: echoing to %s:%u, "
                  "%u packets of %u bytes\n",
                  p_status->udp_session_count + 1u,
                  ip_str,
                  p_status->udp_port,
                  p_status->udp_expected_packets,
                  p_status->udp_payload_size);
    }

    app_main_log_udp_echoes(p_status, p_track);

    if ((0u != p_status->udp_complete) &&
        (p_status->udp_session_count != p_track->summarised_session))
    {
        bool b_has_all_matched = false;

        p_track->summarised_session = p_status->udp_session_count;
        app_main_fmt_ipv4(ip_str, p_status->udp_target);
        b_has_all_matched           =
            ((p_status->udp_packets_matched ==
              p_status->udp_expected_packets) &&
             (0u == p_status->udp_corrupt));

        /* Range-safe: ASCII literal bytes map identically to const UB *. */
        tm_printf((const UB *)"[udp] %s:%u  sent=%u recv=%u matched=%u/%u  "
                  "corrupt=%u retries=%u errs=%u  %u ms\n",
                  ip_str,
                  p_status->udp_port,
                  p_status->udp_packets_sent,
                  p_status->udp_packets_received,
                  p_status->udp_packets_matched,
                  p_status->udp_expected_packets,
                  p_status->udp_corrupt,
                  p_status->udp_retries,
                  p_status->udp_send_errors,
                  p_status->udp_elapsed_ms);
        /* Range-safe: ASCII literal bytes map identically to const UB *. */
        tm_printf((const UB *)"[udp] session %u complete: result=%d%s\n",
                  p_status->udp_session_count + 1u,
                  p_status->udp_result,
                  b_has_all_matched ? "  ALL PACKETS ECHOED" : "");
    }
}

/**
 * @brief Periodically poll CYW43 radio status snapshots and log transitions.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
static void app_main_wifi_status_task (INT stacd, void * p_exinf)
{
    app_main_wifi_track_t track =
    {
        .last_link          = APP_MAIN_WIFI_UNSET_VAL,
        .last_dhcp          = APP_MAIN_WIFI_UNSET_VAL,
        .echo_cursor        = 0u,
        .stalled_polls      = 0u,
        .shown_session      = APP_MAIN_WIFI_UNSET_VAL,
        .summarised_session = APP_MAIN_WIFI_UNSET_VAL
    };

    /* Range-safe: intentionally unused RTOS task entry parameters. */
    (void)stacd;
    /* Range-safe: intentionally unused RTOS extended info pointer. */
    (void)p_exinf;

    app_main_wait_usb_enum(APP_MAIN_WIFI_SETTLE_MSEC);

    for (;;)
    {
        T_CYW43_UTK_STATUS status     = { 0u };
        uint32_t           delay_msec = APP_MAIN_WIFI_IDLE_MSEC;

        cyw43_utk_get_status(&status);
        app_main_log_wifi_link(&status, &track);

        if (0u != status.udp_enabled)
        {
            app_main_log_udp_state(&status, &track);

            if (0u != status.link_up)
            {
                delay_msec = APP_MAIN_WIFI_BUSY_MSEC;
            }
        }

        /* Range-safe: periodic status polling delay. */
        (void)tk_dly_tsk(delay_msec);
    }
}
#endif /* TM_WIFI_CYW43 */

/*** end of file ***/
