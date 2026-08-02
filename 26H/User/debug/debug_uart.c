#include "debug_uart.h"
#include "gm6020_can.h"
#include "pid.h"
#include "flash_params.h"
#include "vision_uart.h"
#include "qiu_weizhi.h"
#include "bno085_shtp.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint8_t          dbg_rx_byte = 0;
uint8_t          dbg_rx_buf[DBG_RX_BUF_SIZE];
volatile uint8_t dbg_rx_idx = 0;
volatile uint8_t  dbg_cmd_ready = 0;
volatile uint8_t  dbg_print_pending = 0;
volatile uint32_t dbg_print_lost = 0;
/* 对齐 ytbeif：默认静音，明确发送 SP1 后才持续输出调参帧。 */
volatile uint8_t  dbg_stream_enable = 0U;

/* 角度标定模式。0=默认输出位置环数据；1=只打原始角度/视觉注释行。 */
volatile uint8_t  dbg_show_angle = 0;

/* main.c 里的调试变量和PID结构体 */
extern volatile float    mubiao_sudu;
extern volatile uint8_t  dianji_shineng;
extern volatile int16_t  dianya_shuchu;
extern volatile uint32_t fasong_count;
extern PID_t             sudu_pid;
extern PID_t             jiaodu_pid;
extern volatile uint8_t  jiaodu_moshi;
extern volatile float    mubiao_qingjiao;
extern volatile float    mubiao_raw_f;
extern volatile float    jiaodu_rpm_cmd;
extern volatile float    competition_target_rpm;
extern volatile float    competition_target_accel_peak_rpm_s;
extern volatile int8_t   zhuanxiang;    // 正转速让raw变大=1，变小=-1
extern volatile int8_t   xianwei_zt;    // 0=行程内 1=撞上限 -1=撞下限
extern volatile uint32_t xianwei_cnt;   // 限位触发累计次数
extern BNO085_SHTP_Data_t bno085_shtp_data;

/* printf 重定向。syscalls.c 里 __io_putchar 是 weak 的，
 * 这里给出强定义就把 printf 接到串口1上了。
 *
 * ⚠️ 不用 HAL_UART_Transmit：它每个字节都等 TC 标志
 *    （整字节含停止位移出才置位），等于完全没有流水线，
 *    再加上 BUSY_TX 状态切换和 HAL_GetTick 开销，
 *    单字节要 3~4 个字符时间。22字节的数据行能拖到 5ms+
 *
 * 直接写 DR 只等 TXE（发送寄存器空，此时上一字节还在移位），
 * 这样下一字节能立刻装填，真正跑满 115200 = 87us/字节
 *
 * 只碰 TX 相关寄存器，不动 RxState，所以和 Receive_IT 不冲突 */
int __io_putchar(int ch)
{
    /* 限次自旋而不是死等：硬件真出问题时不至于卡死主循环。
     * 8400 次约 100us @168MHz，远超一个字节 87us 的时间 */
    uint32_t spin = 8400U;
    while (!(huart1.Instance->SR & UART_FLAG_TXE))
    {
        if (--spin == 0U) return ch;   // 超时放弃这个字节
    }
    huart1.Instance->DR = (uint8_t)ch;
    return ch;
}

/* printf 的行缓冲区。用静态数组而不是让 newlib 自己 malloc：
 * 链接脚本里 _Min_Heap_Size 只有 0x200(512字节)，
 * newlib 默认想申请 1024 字节的 stdout 缓冲，会失败并退化成
 * "每个字符调一次 _write"，效率极差 */
static char stdout_buf[128];
/* Q 查询按参考项目一次性组帧发送，避免多次 printf 与周期帧交错。 */
static char dbg_param_buf[1400];

void Debug_RxStart(void)
{
    /* 行缓冲：攒够一行(遇到\n)才一次性吐出去，
     * 避免逐字符 _write 的调用开销 */
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

    HAL_UART_Receive_IT(&huart1, &dbg_rx_byte, 1);
}

/* 单字节接收回调。只做攒字符，解析放主循环——
 * 中断里做 atof 和字符串处理太重，会挤占2ms速度环 */
void Debug_RxByte(void)
{
    uint8_t ch = dbg_rx_byte;

    if (ch == '\r' || ch == '\n')
    {
        /* 上一条还没处理完就丢弃新的，避免解析途中缓冲被改 */
        if (dbg_rx_idx > 0U && !dbg_cmd_ready)
        {
            dbg_rx_buf[dbg_rx_idx] = '\0';
            dbg_cmd_ready = 1U;
        }
        dbg_rx_idx = 0U;
    }
    else if (!dbg_cmd_ready)
    {
        if (dbg_rx_idx < (DBG_RX_BUF_SIZE - 1U))
        {
            dbg_rx_buf[dbg_rx_idx++] = ch;
        }
        else
        {
            dbg_rx_idx = 0U;   // 超长命令直接丢，防越界
        }
    }

    HAL_UART_Receive_IT(&huart1, &dbg_rx_byte, 1);   // 重新武装
}

/* ── 输出分两种模式，A 命令切换，互斥 ─────────────────────────
 *
 * A1 角度标定模式：
 *     只有 100ms 一条的角度行，PID数据行和诊断行全部不打，
 *     手动转云台读数不会被刷屏干扰
 *       # raw=3421 ang=150.3 turn=0
 *
 * A0 位置环调参模式（默认）：
 *     100ms 数据行，见下
 *
 * ── 数据行，100ms 一条（A0 模式）──────────────────────────────
 *
 * 格式：label:value 逗号分隔，标签全英文
 *
 *   qtgt:0.0,qpos:12.3,qerr:-12.3,qvel:1.25,qcmd:-0.62,
 *   qkp:0.050,qki:0.0000,qkd:0.080,qon:1,conn:1
 *
 * 绘图软件靠标签对应通道，所以标签名和顺序都别改。
 * 诊断行用 # 开头且用 = 号，和数据行区分开，不会被当成通道
 *
 * qtgt/qpos/qerr/qvel/qcmd 是位置环的目标、反馈、误差、速度和输出；
 * qkp/qki/qkd 是当前正在调的增益；qon/conn 用于确认环路和视觉在线。
 * 温度、CAN、视觉帧计数等诊断量单独走 # 注释行，绘图工具会自动忽略。
 * ─────────────────────────────────────────────────────────── */
void dianji_dayin(void)
{
    /* 与参数查询彻底解耦：即使旧 pending 或其他调用点误触发，
     * SP0 状态下也绝不输出周期数据帧。 */
    if (!dbg_stream_enable)
    {
        return;
    }

    /* ── 角度标定模式：只打角度，其他全静音 ──────────────────
     * 本函数由位置环10ms打印节拍触发。
     * 全部 # 开头，绘图工具当注释跳过 */
    if (dbg_show_angle == 1U)
    {
        /* 一次性快照，避免打印中途被CAN中断改掉导致三个值不自洽 */
        int16_t  yuanshi   = motor_date[0].rotor_angle;
        float    du        = motor_date[0].rotor_du_angle;
        int32_t  quanshu   = motor_date[0].turn_count;

        int8_t   zt        = xianwei_zt;

        /* raw/ang/turn/lim/dn/up : 限位联调
         * vis/conn/vrx           : 视觉 USART2→解析结果 */
        printf("# raw=%d ang=%.1f turn=%ld lim=%s dn=%d up=%d"
               " vis=%.2f conn=%u vrx=%lu qon=%u qcmd=%.2f\r\n",
               (int)yuanshi, du, (long)quanshu,
               (zt > 0 ? "HI" : (zt < 0 ? "LO" : "OK")),
               (int)(yuanshi - JIAODU_RUAN_MIN),
               (int)(JIAODU_RUAN_MAX - yuanshi),
               vision.distance_mm,
               (unsigned)vision.connected,
               (unsigned long)vision.rx_count,
               (unsigned)qiu_moshi,
               (double)qiu_qingjiao_cmd);
        return;
    }

    if (dbg_show_angle == 2U)
    {
        uint32_t imu_age = (bno085_shtp_data.accel_last_ms == 0U)
                         ? 9999U
                         : (HAL_GetTick() - bno085_shtp_data.accel_last_ms);

        printf("atgt:%.2f,adeg:%.2f,qpos:%.2f,qvel:%.1f,"
               "ax:%.3f,ay:%.3f,az:%.3f,iage:%lu,ist:%u,ierr:%lu,ff:%.2f,"
               "iraw:%lu,ipkt:%u,ibad:%u,irxs:%u,itx:%u\r\n",
               (double)mubiao_qingjiao,
               (double)(((float)motor_date[0].rotor_angle -
                          (float)jiaodu_pingheng_raw) / JIAODU_TICK_PER_DEG),
               (double)(qiu_mubiao_mm - qiu_wucha_mm),
               (double)qiu_su_mm,
               (double)bno085_shtp_data.accel_x,
               (double)bno085_shtp_data.accel_y,
               (double)bno085_shtp_data.accel_z,
               (unsigned long)imu_age,
               (unsigned)bno085_shtp_data.state,
               (unsigned long)bno085_shtp_data.uart_error,
               (double)qiu_ff_cmd,
               (unsigned long)bno085_shtp_data.raw_bytes,
               (unsigned)bno085_shtp_data.rx_packets,
               (unsigned)bno085_shtp_data.rx_bad,
               (unsigned)bno085_shtp_data.rx_status,
               (unsigned)bno085_shtp_data.tx_space);
        return;
    }

    /* 位置环标定帧，10Hz输出：闭环、补偿、执行器三段同时可观测。
     * qbase=球位置/球速闭环输出，curve/imu=附加前馈，qcmd=最终倾角；
     * atgt/adeg、raw/rpm/trpm/volt 用于判断平台是否跟上该命令；
     * cacc为最近打印窗口内底盘目标加速度的带符号峰值，避免漏掉20ms阶跃。 */
    float chassis_accel_peak_rpm_s;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    chassis_accel_peak_rpm_s = competition_target_accel_peak_rpm_s;
    competition_target_accel_peak_rpm_s = 0.0f;
    if (primask == 0U)
    {
        __enable_irq();
    }
    printf("qpos:%.1f,qerr:%.1f,qvel:%.0f,qtv:%.0f,qiv:%.1f,"
           "qbase:%.2f,curve:%.2f,imu:%.2f,qcmd:%.2f,"
           "atgt:%.2f,adeg:%.2f,raw:%d,rpm:%d,trpm:%.1f,volt:%d,"
           "vage:%lu,odt:%.0f,win:%.0f,crpm:%.1f,cacc:%.1f\r\n",
           (double)(qiu_mubiao_mm - qiu_wucha_mm),
           (double)qiu_wucha_mm,
           (double)qiu_su_mm,
           (double)qiu_mubiao_v,
           (double)qiu_jifen_v,
           (double)qiu_base_cmd,
           (double)qiu_guan_ff_cmd,
           (double)qiu_ff_cmd,
           (double)qiu_qingjiao_cmd,
           (double)mubiao_qingjiao,
           (double)(((float)motor_date[0].rotor_angle -
                      (float)jiaodu_pingheng_raw) / JIAODU_TICK_PER_DEG),
           (int)motor_date[0].rotor_angle,
           (int)motor_date[0].rotor_speed,
           (double)jiaodu_rpm_cmd,
           (int)dianya_shuchu,
           (unsigned long)((vision.last_rx_ms == 0U)
                           ? 9999U : (HAL_GetTick() - vision.last_rx_ms)),
           (double)qiu_waibu_dt_ms,
           (double)qiu_lishi_kuadu_ms,
           (double)competition_target_rpm,
           (double)chassis_accel_peak_rpm_s);

    /* 以下为温度、CAN、限位、视觉等每秒诊断；当前内环测试不打印。
     * 需要排查总线或视觉故障时，再恢复这一段。 */
    /*
    static uint8_t manpai = 0;
    uint32_t can_rx = motor_date[0].rx_count;
    if (++manpai >= 100U)
    {
        manpai = 0;
        printf("# temp=%u rx=%lu txerr=%lu en=%u lost=%lu tx=%lu\r\n"
               "# raw=%d lim=%s hit=%lu dir=%d volt=%d\r\n"
               "# vis=%.2f conn=%u ok=%u vrx=%lu verr=%lu\r\n",
               (unsigned)motor_date[0].motor_temperature,
               (unsigned long)can_rx,
               (unsigned long)can_tx_err_count,
               (unsigned)dianji_shineng,
               (unsigned long)dbg_print_lost,
               (unsigned long)fasong_count,
               (int)motor_date[0].rotor_angle,
               (xianwei_zt > 0 ? "HI" : (xianwei_zt < 0 ? "LO" : "OK")),
               (unsigned long)xianwei_cnt,
               (int)zhuanxiang,
               (int)dianya_shuchu,
               vision.distance_mm,
               (unsigned)vision.connected,
               (unsigned)vision.valid,
               (unsigned long)vision.rx_count,
               (unsigned long)vision.err_count);

        if (dianji_shineng && can_rx == 0U)
        {
            printf("# WARN: en=1 but CAN rx=0, speed loop locked (no feedback)\r\n");
            printf("# check: motor power, CANH/CANL, 120R, ID1->0x205\r\n");
        }
        if (fasong_count == 0U)
        {
            printf("# WARN: tx=0, TIM6 not running?\r\n");
        }
        if (qiu_moshi && !vision.connected)
        {
            printf("# WARN: qiu on but vision offline, tilt easing to 0\r\n");
        }
    }
    */
}

void Debug_PrintParams(void)
{
    int len;
    uint16_t tx_len;

    /* 全部 # 开头：绘图工具当注释跳过，不会污染曲线 */
    len = snprintf(dbg_param_buf, sizeof(dbg_param_buf),
                   "# PARAM spd Kp=%.2f Ki=%.2f Kd=%.2f | ang Kp=%.3f Ki=%.3f Kd=%.3f\r\n"
                   "# STATE mode=%u tdeg=%.1f traw=%.0f raw=%d rpm=%.1f en=%u volt=%d\r\n"
                   "# LIMIT soft=%d~%d hard=%d~%d zero=%ld work=±%.1fdeg lim=%s dir=%d\r\n"
                   "# VIS   mm=%.2f conn=%u ok=%u rx=%lu err=%lu\r\n"
                   "# QIU   on=%u pos X=%.2f Y=%.3f Z=%.2f vel C=%.3f O=%.3f\r\n"
                   "# QIU   tgt=%.1fmm err=%.1fmm v=%.0f tv=%.0f tilt=±%.1f cmd=%.2f\r\n"
                   "# QIU   rate=%.0f dead=%.1f sign=%d lost=%lu\r\n"
                   "# QIU   ff=%u phys=%u K=%.2fdeg/(m/s2) lim=%.1fdeg sign=%d a=%.3f cmd=%.2f\r\n"
                   "# QIU   curve K=%.3fdeg/mm cmd=%.2fdeg\r\n"
                   "# QIU   outer=%lu dt=%.1fms win=%.1fms (vision/2, fit4)\r\n"
                   "# CMD   T V E S | J/K/L ang | P/I/D spd | PH<deg> zero-cal"
                   " | B/X/Y/Z/C/O/G/H/U/M/N qiu | SP<0/1> stream | SAVE/W R Q\r\n"
                   "# DASH  on=%u QK=%.1f QW=%lums QC=%lums hold=%u\r\n"
                   "# CMD   QD<0/1> QK<k> QW<ms> QC<ms>\r\n",
                   sudu_pid.Kp, sudu_pid.Ki, sudu_pid.Kd,
                   jiaodu_pid.Kp, jiaodu_pid.Ki, jiaodu_pid.Kd,
                   (unsigned)jiaodu_moshi, mubiao_qingjiao, mubiao_raw_f,
                   (int)motor_date[0].rotor_angle, mubiao_sudu,
                   (unsigned)dianji_shineng, dianya_shuchu,
                   JIAODU_RUAN_MIN, JIAODU_RUAN_MAX,
                   JIAODU_MIN, JIAODU_MAX,
                   (long)jiaodu_pingheng_raw, (double)jiaodu_gongzuo_du,
                   (xianwei_zt > 0 ? "HI" : (xianwei_zt < 0 ? "LO" : "OK")),
                   (int)zhuanxiang,
                   vision.distance_mm,
                   (unsigned)vision.connected,
                   (unsigned)vision.valid,
                   (unsigned long)vision.rx_count,
                   (unsigned long)vision.err_count,
                   (unsigned)qiu_moshi,
                   qiu_pid.Kp, qiu_pid.Ki, qiu_pid.Kd,
                   (double)qiu_vel_kp, (double)qiu_vel_kd,
                   (double)qiu_mubiao_mm, (double)qiu_wucha_mm,
                   (double)qiu_su_mm, (double)qiu_mubiao_v,
                   (double)qiu_qingjiao_max, (double)qiu_qingjiao_cmd,
                   (double)qiu_sulv_max, (double)qiu_siqu_mm,
                   (int)qiu_fuhao, (unsigned long)qiu_diushi_cnt,
                   (unsigned)qiu_ff_enable, (unsigned)qiu_ff_physics_enable,
                   (double)qiu_ff_k,
                   (double)qiu_ff_max, (int)qiu_ff_sign,
                   (double)qiu_ff_accel, (double)qiu_ff_cmd,
                   (double)qiu_guan_ff_k, (double)qiu_guan_ff_cmd,
                   (unsigned long)qiu_waibu_cnt,
                   (double)qiu_waibu_dt_ms, (double)qiu_lishi_kuadu_ms,
                   (unsigned)qiu_chongci_moshi, (double)qiu_shache_k,
                   (unsigned long)qiu_kazhu_maichong_ms,
                   (unsigned long)qiu_kazhu_lengque_ms,
                   (unsigned)qiu_daoda_baochi);

    if (len <= 0)
    {
        return;
    }
    tx_len = (len < (int)sizeof(dbg_param_buf))
           ? (uint16_t)len
           : (uint16_t)(sizeof(dbg_param_buf) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)dbg_param_buf, tx_len, 300U);
}

/* 主循环里轮询。命令格式：字母+数值，如 P50 / V-30 / E1
 * 另支持整词：SAVE（与 ytbeif 一致） */
void Debug_ParseCommand(void)
{
    if (!dbg_cmd_ready) return;

    char *cmd = (char *)dbg_rx_buf;
    float val;

    /* ytbeif 同款：整词 SAVE 写入 Flash（也认 W） */
    if (strcmp(cmd, "SAVE") == 0 || strcmp(cmd, "save") == 0)
    {
        (void)FlashParams_Save(&sudu_pid, &jiaodu_pid, zhuanxiang);
        mubiao_raw_f = (float)jiaodu_pingheng_raw +
                       mubiao_qingjiao * JIAODU_TICK_PER_DEG;
        dbg_cmd_ready = 0;
        return;
    }
    if (strcmp(cmd, "LOAD") == 0 || strcmp(cmd, "load") == 0)
    {
        if (FlashParams_Load(&sudu_pid, &jiaodu_pid, &zhuanxiang))
        {
            mubiao_raw_f = (float)jiaodu_pingheng_raw +
                           mubiao_qingjiao * JIAODU_TICK_PER_DEG;
        }
        dbg_cmd_ready = 0;
        return;
    }

    /* SP0/SP1：只关闭周期数据行，参数查询和命令回显始终保留。
     * 必须在单字母 S 急停之前截获，避免 SP0 被误判成急停。 */
    if ((cmd[0] == 'S' || cmd[0] == 's') &&
        (cmd[1] == 'P' || cmd[1] == 'p'))
    {
        float stream_val = atof(&cmd[2]);
        dbg_stream_enable = (stream_val >= 1.0f) ? 1U : 0U;
        if (!dbg_stream_enable)
        {
            dbg_print_pending = 0U;
        }
        printf("# STREAM on=%u (Q/SET replies stay on)\r\n",
               (unsigned)dbg_stream_enable);
        dbg_cmd_ready = 0;
        return;
    }

    /* PH<度>：水平零点标定。含义="现在 T<度> 时平台才真水平"，
     * 把这个度数吃进零点，标完 T0 就是真水平。
     * ⚠️ 必须截在 atof 之前：否则 PH1.6 掉进 case 'P'，
     *    atof("H1.6")=0 会把速度环 Kp 直接打成 0。
     * 累加语义：零点偏了再量个小值发一次就行，不用算绝对 raw。
     * 改完要 W 存档，否则下次开机 FlashParams_Load 又盖回旧值。 */
    if ((cmd[0] == 'P' || cmd[0] == 'p') &&
        (cmd[1] == 'H' || cmd[1] == 'h'))
    {
        float du  = atof(&cmd[2]);
        int32_t xin = jiaodu_pingheng_raw +
                      (int32_t)(du * JIAODU_TICK_PER_DEG + (du >= 0.0f ? 0.5f : -0.5f));
        int32_t ban = (int32_t)(jiaodu_gongzuo_du * JIAODU_TICK_PER_DEG);

        /* 挪零点等于挪整个工作窗口，先确认两端不会顶到软限位 */
        if ((xin - ban) < JIAODU_RUAN_MIN || (xin + ban) > JIAODU_RUAN_MAX)
        {
            printf("# ERR zero=%ld would push work window %ld~%ld"
                   " outside soft %d~%d\r\n",
                   (long)xin, (long)(xin - ban), (long)(xin + ban),
                   JIAODU_RUAN_MIN, JIAODU_RUAN_MAX);
        }
        else
        {
            jiaodu_pingheng_raw = xin;
            /* 零点一动，之前攒的积分都是按旧零点算的，全清掉 */
            PID_Reset(&jiaodu_pid);
            QiuWeizhi_Reset();
            mubiao_qingjiao = 0.0f;
            mubiao_raw_f    = (float)jiaodu_pingheng_raw;
            printf("# SET zero %+.2fdeg -> raw=%ld (T0 now true level)\r\n"
                   "# work %ld~%ld soft %d~%d | send W to keep it\r\n",
                   (double)du, (long)jiaodu_pingheng_raw,
                   (long)(xin - ban), (long)(xin + ban),
                   JIAODU_RUAN_MIN, JIAODU_RUAN_MAX);
        }
        dbg_cmd_ready = 0;
        return;
    }

    /* FE/FP/FK/FL/FS：IMU前馈使能、物理模式、增益、限幅、方向。 */
    if ((cmd[0] == 'F' || cmd[0] == 'f') &&
        (cmd[1] == 'E' || cmd[1] == 'e' ||
         cmd[1] == 'P' || cmd[1] == 'p' ||
         cmd[1] == 'K' || cmd[1] == 'k' ||
         cmd[1] == 'L' || cmd[1] == 'l' ||
         cmd[1] == 'S' || cmd[1] == 's'))
    {
        float ff_val = atof(&cmd[2]);
        char ff_cmd = cmd[1];

        if (ff_cmd == 'E' || ff_cmd == 'e')
        {
            qiu_ff_enable = (ff_val >= 1.0f) ? 1U : 0U;
        }
        else if (ff_cmd == 'P' || ff_cmd == 'p')
        {
            qiu_ff_physics_enable = (ff_val >= 1.0f) ? 1U : 0U;
        }
        else if (ff_cmd == 'K' || ff_cmd == 'k')
        {
            qiu_ff_k = PID_LIMIT_MIN_MAX(ff_val, 0.0f, 8.0f);
        }
        else if (ff_cmd == 'L' || ff_cmd == 'l')
        {
            qiu_ff_max = PID_LIMIT_MIN_MAX(ff_val, 0.0f,
                                           jiaodu_gongzuo_du);
        }
        else if (ff_cmd == 'S' || ff_cmd == 's')
        {
            qiu_ff_sign = (ff_val < 0.0f) ? -1 : 1;
        }
        QiuWeizhi_FFReset();
        printf("# FF en=%u phys=%u K=%.2f lim=%.1f sign=%d\r\n",
               (unsigned)qiu_ff_enable,
               (unsigned)qiu_ff_physics_enable, (double)qiu_ff_k,
               (double)qiu_ff_max, (int)qiu_ff_sign);
        dbg_cmd_ready = 0;
        return;
    }

    /* QD/QK/QW/QC：H3冲刺模式那一套旋钮。
     * ⚠️ 同样必须截在 atof 之前，否则 QD1 掉进单字母 case 'Q'。
     * 调试顺序建议：负向到不了位先加 QW / 减 QC（脱困占空比），
     * 冲过头减 QK（刹车曲线）。 */
    if ((cmd[0] == 'Q' || cmd[0] == 'q') &&
        (cmd[1] == 'D' || cmd[1] == 'd' ||
         cmd[1] == 'K' || cmd[1] == 'k' ||
         cmd[1] == 'W' || cmd[1] == 'w' ||
         cmd[1] == 'C' || cmd[1] == 'c'))
    {
        float qiu_val = atof(&cmd[2]);
        char  qiu_cmd = cmd[1];

        if (qiu_cmd == 'D' || qiu_cmd == 'd')
        {
            QiuWeizhi_SetDashMode((qiu_val >= 1.0f) ? 1U : 0U);
        }
        else if (qiu_cmd == 'K' || qiu_cmd == 'k')
        {
            qiu_shache_k = PID_LIMIT_MIN_MAX(qiu_val, 8.0f, 80.0f);
        }
        else if (qiu_cmd == 'W' || qiu_cmd == 'w')
        {
            qiu_kazhu_maichong_ms =
                (uint32_t)PID_LIMIT_MIN_MAX(qiu_val, 40.0f, 400.0f);
        }
        else if (qiu_cmd == 'C' || qiu_cmd == 'c')
        {
            qiu_kazhu_lengque_ms =
                (uint32_t)PID_LIMIT_MIN_MAX(qiu_val, 0.0f, 600.0f);
        }
        printf("# DASH on=%u K=%.1f pulse=%lums cool=%lums\r\n",
               (unsigned)qiu_chongci_moshi, (double)qiu_shache_k,
               (unsigned long)qiu_kazhu_maichong_ms,
               (unsigned long)qiu_kazhu_lengque_ms);
        dbg_cmd_ready = 0;
        return;
    }

    val = atof(&cmd[1]);       // 无数字时返回0，正好当默认值

    switch (cmd[0])
    {
        case 'P': case 'p':
            sudu_pid.Kp = val;
            printf("# SET Kp=%.2f\r\n", sudu_pid.Kp);
            break;

        case 'I': case 'i':
            sudu_pid.Ki = val;
            /* 必须清积分：旧err_sum是按老Ki累的，
             * 直接换增益输出会瞬间跳一大截，电机猛窜 */
            sudu_pid.err_sum = 0.0f;
            printf("# SET Ki=%.2f (isum cleared)\r\n", sudu_pid.Ki);
            break;

        case 'D': case 'd':
            sudu_pid.Kd = val;
            printf("# SET Kd=%.2f\r\n", sudu_pid.Kd);
            break;

        case 'V': case 'v':
            /* 纯速度模式：两个外环都让路 */
            qiu_moshi = 0;
            QiuWeizhi_Reset();
            jiaodu_moshi = 0;
            PID_Reset(&jiaodu_pid);
            jiaodu_rpm_cmd = 0.0f;
            mubiao_sudu = val;
            printf("# SPEED mode tgt=%.1f rpm (qiu off)\r\n", mubiao_sudu);
            break;

        case 'T': case 't':
            /* 角度模式：T=相对水平倾角(度)，内部→raw→角度环→RPM
             * 手动给倾角 = 明确要接管，所以顺手关掉位置环，
             * 否则位置环下一拍就把你的 T 值覆盖掉，看着像"命令没生效" */
            if (qiu_moshi)
            {
                qiu_moshi = 0;
                QiuWeizhi_Reset();
                printf("# qiu off (manual T)\r\n");
            }
            if (val > jiaodu_gongzuo_du)  val = jiaodu_gongzuo_du;
            if (val < -jiaodu_gongzuo_du) val = -jiaodu_gongzuo_du;
            mubiao_qingjiao = val;
            mubiao_raw_f = (float)jiaodu_pingheng_raw +
                           mubiao_qingjiao * JIAODU_TICK_PER_DEG;
            if (!jiaodu_moshi)
            {
                PID_Reset(&jiaodu_pid);
                PID_Reset(&sudu_pid);
            }
            jiaodu_moshi = 1;
            printf("# ANGLE mode T=%.1fdeg raw_tgt=%.0f (zero=%ld ±%.1f)\r\n",
                   mubiao_qingjiao, mubiao_raw_f,
                   (long)jiaodu_pingheng_raw, (double)jiaodu_gongzuo_du);
            break;

        case 'J': case 'j':
            jiaodu_pid.Kp = val;
            printf("# SET ang Kp=%.3f\r\n", jiaodu_pid.Kp);
            break;

        case 'K': case 'k':
            jiaodu_pid.Ki = val;
            jiaodu_pid.err_sum = 0.0f;
            printf("# SET ang Ki=%.3f (isum cleared)\r\n", jiaodu_pid.Ki);
            break;

        case 'L': case 'l':
            jiaodu_pid.Kd = val;
            printf("# SET ang Kd=%.3f\r\n", jiaodu_pid.Kd);
            break;

        /* ── 球串级：位置10ms→目标球速，球速10ms→倾角 ───────── */
        case 'B': case 'b':
            if (val >= 1.0f)
            {
                QiuWeizhi_Reset();
                PID_Reset(&jiaodu_pid);
                jiaodu_moshi = 1;
                qiu_moshi    = 1;
                printf("# QIU on cascade: pos X=%.2f Y=%.3f Z=%.2f"
                       " vel C=%.3f O=%.3f G=%.1fmm H=±%.1f M=%.0f N=%d\r\n",
                       qiu_pid.Kp, qiu_pid.Ki, qiu_pid.Kd,
                       (double)qiu_vel_kp, (double)qiu_vel_kd,
                       (double)qiu_mubiao_mm, (double)qiu_qingjiao_max,
                       (double)qiu_sulv_max, (int)qiu_fuhao);
                if (!vision.connected)
                {
                    printf("# WARN vision offline, tilt held at 0 until frames\r\n");
                }
                if (!dianji_shineng)
                {
                    printf("# WARN motor off, send E1\r\n");
                }
            }
            else
            {
                qiu_moshi = 0;
                QiuWeizhi_Reset();
                printf("# QIU off (angle mode kept, T takes over)\r\n");
            }
            break;

        case 'X': case 'x':
            /* 位置环 Kp：(mm/s)/mm，大了回中快、易过冲 */
            qiu_pid.Kp = val;
            printf("# SET pos Kp=%.3f (mm/s)/mm\r\n", qiu_pid.Kp);
            break;

        case 'Y': case 'y':
            qiu_pid.Ki = val;
            qiu_pid.err_sum = 0.0f;
            printf("# SET pos Ki=%.4f (isum cleared)\r\n", qiu_pid.Ki);
            break;

        case 'Z': case 'z':
            /* 位置环 Kd：靠近中心提前砍目标速度 */
            qiu_pid.Kd = val;
            printf("# SET pos Kd=%.3f\r\n", qiu_pid.Kd);
            break;

        case 'C': case 'c':
            /* 球速环 Kp：°/(mm/s)，速度跟不上就加大 */
            qiu_vel_kp = val;
            printf("# SET vel Kp=%.4f deg/(mm/s)\r\n", (double)qiu_vel_kp);
            break;

        case 'O': case 'o':
            /* 球速环阻尼 Kd：冲过中心就加大 */
            qiu_vel_kd = val;
            printf("# SET vel Kd=%.4f\r\n", (double)qiu_vel_kd);
            break;

        case 'G': case 'g':
            /* 目标球位置 mm。想让球停偏一点就给非 0 */
            qiu_mubiao_mm = val;
            printf("# SET qiu tgt=%.1fmm\r\n", (double)qiu_mubiao_mm);
            break;

        case 'H': case 'h':
        {
            float raw_lo;
            float raw_hi;

            /* 输出倾角限幅；需要放大工作半宽时先校验编码器软限位。 */
            if (val < 0.1f)  val = 0.1f;
            raw_lo = (float)jiaodu_pingheng_raw - val * JIAODU_TICK_PER_DEG;
            raw_hi = (float)jiaodu_pingheng_raw + val * JIAODU_TICK_PER_DEG;
            if (raw_lo < (float)JIAODU_RUAN_MIN || raw_hi > (float)JIAODU_RUAN_MAX)
            {
                printf("# ERR H=%.1fdeg needs raw %.0f~%.0f outside soft %d~%d\r\n",
                       (double)val, (double)raw_lo, (double)raw_hi,
                       JIAODU_RUAN_MIN, JIAODU_RUAN_MAX);
                break;
            }
            if (val > jiaodu_gongzuo_du)
            {
                jiaodu_gongzuo_du = val;
            }
            qiu_qingjiao_max = val;
            printf("# SET qiu tilt=±%.1fdeg (work=±%.1f)\r\n",
                   (double)qiu_qingjiao_max, (double)jiaodu_gongzuo_du);
            break;
        }

        case 'U': case 'u':
            /* 位置死区 mm，0=关（默认）。球在目标附近微颤才考虑开 */
            if (val < 0.0f) val = 0.0f;
            qiu_siqu_mm = val;
            printf("# SET qiu dead=%.1fmm%s\r\n",
                   (double)qiu_siqu_mm, (val > 0.0f) ? "" : " (off)");
            break;

        case 'M': case 'm':
            /* 倾角变化率限幅 °/s。越小越"稳但肉" */
            if (val < 1.0f) val = 1.0f;
            qiu_sulv_max = val;
            printf("# SET qiu rate=%.0fdeg/s (%.2fdeg per 5ms)\r\n",
                   (double)qiu_sulv_max,
                   (double)(qiu_sulv_max * QIU_SERVICE_ZHOUQI_S));
            break;

        case 'N': case 'n':
            /* 推球方向标定：球一路跑到底不回头就敲反的 */
            qiu_fuhao = (val < 0.0f) ? (int8_t)-1 : (int8_t)1;
            QiuWeizhi_Reset();
            printf("# SET qiu sign=%d (loop reset)\r\n", (int)qiu_fuhao);
            break;

        case 'E': case 'e':
            if (val >= 1.0f)
            {
                PID_Reset(&sudu_pid);
                PID_Reset(&jiaodu_pid);
                dianji_shineng = 1;
                printf("# MOTOR ON  can_rx=%lu tx=%lu raw=%d mode=%u\r\n",
                       (unsigned long)motor_date[0].rx_count,
                       (unsigned long)fasong_count,
                       (int)motor_date[0].rotor_angle,
                       (unsigned)jiaodu_moshi);
                if (motor_date[0].rx_count == 0U)
                {
                    printf("# WARN no CAN feedback yet; loop stays disarmed until rx>0\r\n");
                }
            }
            else
            {
                dianji_shineng = 0;
                printf("# MOTOR OFF\r\n");
            }
            break;

        case 'S': case 's':
            dianji_shineng = 0;
            mubiao_sudu    = 0.0f;
            jiaodu_rpm_cmd = 0.0f;
            /* 急停：位置环直接关掉。留着它，E1 一按球就又跑起来了 */
            qiu_moshi = 0;
            QiuWeizhi_Reset();
            /* 不强制改角度/速度模式，但清所有积分 */
            PID_Reset(&sudu_pid);
            PID_Reset(&jiaodu_pid);
            printf("# !! STOP !! (qiu off)\r\n");
            break;

        case 'A': case 'a':
            /* A0位置环、A1角度标定、A2 IMU标定，三种输出互斥。 */
            if (val >= 2.0f)
            {
                dbg_show_angle = 2;
                printf("# IMU MODE on: ax/ay/az/iage/ist/ierr/ff; A0 restores\r\n");
            }
            else if (val >= 1.0f)
            {
                dbg_show_angle = 1;
                printf("# ANGLE MODE on (100ms), pid data muted\r\n");
            }
            else
            {
                dbg_show_angle = 0;
                printf("# ANGLE MODE off, pid data resumed\r\n");
            }
            break;

        case 'F': case 'f':
            /* 转向标定：正转速让 raw 变大填1，变小填-1。
             * 填反了限位会拦反方向，表现是"往外拦不住、往里回不去" */
            zhuanxiang = (val < 0.0f) ? -1 : 1;
            mubiao_sudu = 0.0f;          // 改方向前先停，免得判断瞬间翻转
            printf("# SET dir=%d (tgt zeroed)\r\n", (int)zhuanxiang);
            break;

        case 'W': case 'w':
            /* 单字母同 SAVE */
            (void)FlashParams_Save(&sudu_pid, &jiaodu_pid, zhuanxiang);
            break;

        case 'R': case 'r':
            /* 单字母同 LOAD */
            if (FlashParams_Load(&sudu_pid, &jiaodu_pid, &zhuanxiang))
            {
                mubiao_raw_f = (float)jiaodu_pingheng_raw +
                               mubiao_qingjiao * JIAODU_TICK_PER_DEG;
            }
            break;

        case 'Q': case 'q':
            Debug_PrintParams();
            break;

        default:
            printf("# ERR use: T V E S | P I D | J K L | B X Y Z G H U M N"
                   " | PH<deg> zero-cal | SAVE/W LOAD/R A F Q\r\n");
            break;
    }

    dbg_cmd_ready = 0;   // 处理完才放开，期间中断不会覆盖缓冲
}
