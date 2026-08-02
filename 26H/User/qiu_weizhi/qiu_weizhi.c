#include "qiu_weizhi.h"
#include "vision_uart.h"
#include "flash_params.h"
#include "bno085_shtp.h"

#include <math.h>

extern volatile float   mubiao_qingjiao;
extern volatile uint8_t jiaodu_moshi;
extern BNO085_SHTP_Data_t bno085_shtp_data;

typedef struct
{
    float position_mm;
    uint32_t timestamp_ms;
} Qiu_LishiYangben_t;

PID_t qiu_pid;

volatile uint8_t  qiu_moshi        = 0U;
volatile float    qiu_mubiao_mm    = QIU_MUBIAO_MM;
volatile float    qiu_qingjiao_max = QIU_QINGJIAO_MAX;
volatile float    qiu_siqu_mm      = QIU_SIQU_MM;
volatile float    qiu_sulv_max     = QIU_SULV_MAX;
volatile int8_t   qiu_fuhao        = 1;

volatile float    qiu_vel_kp       = QIU_VEL_KP_DEFAULT;
volatile float    qiu_vel_kd       = QIU_VEL_KD_DEFAULT;
volatile float    qiu_kazhu_qingjiao = QIU_KAZHU_QINGJIAO;
volatile float    qiu_kazhu_qingjiao_fu = QIU_KAZHU_QINGJIAO;

volatile float    qiu_wucha_mm       = 0.0f;
volatile float    qiu_su_mm          = 0.0f;
volatile float    qiu_mubiao_v       = 0.0f;
volatile float    qiu_jifen_v        = 0.0f;
volatile float    qiu_qingjiao_cmd   = 0.0f;
volatile float    qiu_base_cmd        = 0.0f;
volatile uint8_t  qiu_ff_enable      = QIU_FF_ENABLE_DEFAULT;
volatile uint8_t  qiu_ff_physics_enable = QIU_FF_PHYSICS_ENABLE_DEFAULT;
volatile float    qiu_ff_k           = QIU_FF_K_DEFAULT;
volatile float    qiu_ff_max         = QIU_FF_MAX_DEFAULT;
volatile int8_t   qiu_ff_sign        = QIU_FF_SIGN_DEFAULT;
volatile float    qiu_ff_accel       = 0.0f;
volatile float    qiu_ff_cmd         = 0.0f;
volatile float    qiu_chassis_cmd_ff = 0.0f;
volatile float    qiu_guan_ff_k      = QIU_GUAN_FF_K_DEFAULT;
volatile float    qiu_guan_ff_cmd    = 0.0f;
volatile uint32_t qiu_diushi_cnt     = 0U;
volatile uint32_t qiu_waibu_cnt      = 0U;
volatile float    qiu_waibu_dt_ms    = 0.0f;
volatile float    qiu_lishi_kuadu_ms = 0.0f;
volatile uint8_t  qiu_kazhu_maichong = 0U;
volatile uint8_t  qiu_daoda_baochi   = 0U;

/* 冲刺模式：H3 全程打开，H5/H6 不开，行为完全不变。 */
volatile uint8_t  qiu_chongci_moshi  = 0U;
volatile float    qiu_shache_k       = QIU_SHACHE_K_DEFAULT;
volatile uint32_t qiu_kazhu_maichong_ms = QIU_KAZHU_MAICHONG_MS;
volatile uint32_t qiu_kazhu_lengque_ms  = QIU_KAZHU_LENGQUE_MS;

static Qiu_LishiYangben_t qiu_lishi[QIU_LISHI_YANGBEN];
static uint8_t  qiu_lishi_count = 0U;
static uint8_t  qiu_waibu_yiqidong = 0U;
static uint8_t  qiu_waibu_zhijian_frame = 0U;
static uint8_t  qiu_jifen_youxiao = 0U;
static uint8_t  qiu_shijue_xin = 0U;
static uint8_t  qiu_rx_yitongbu = 0U;
static uint8_t  qiu_mubiao_yitongbu = 0U;
static uint8_t  qiu_kazhu_count = 0U;
static uint8_t  qiu_kazhu_anchor_valid = 0U;
static uint32_t qiu_shang_rx_count = 0U;
static uint32_t qiu_shang_waibu_ms = 0U;
static float    qiu_shang_waibu_err = 0.0f;
static float    qiu_shang_cmd = 0.0f;
static float    qiu_qingjiao_mubiao = 0.0f;
static float    qiu_qingjiao_jichu = 0.0f;
static float    qiu_shang_mubiao_mm = 0.0f;
static float    qiu_kazhu_anchor_mm = 0.0f;
static float    qiu_ff_filtered = 0.0f;
static uint32_t qiu_ff_last_updates = 0U;
static uint8_t  qiu_fu_maichong_moshi = 0U;
static uint8_t  qiu_kazhu_lengque = 0U;
static uint32_t qiu_kazhu_maichong_start_ms = 0U;
static uint32_t qiu_kazhu_lengque_start_ms = 0U;

static float qiu_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static uint8_t qiu_zhongxin_huizhong_moshi(void)
{
    return !qiu_chongci_moshi && qiu_absf(qiu_mubiao_mm) < 0.05f;
}

static void qiu_kazhu_jiance_qingkong(void)
{
    qiu_kazhu_count = 0U;
    qiu_kazhu_anchor_valid = 0U;
    qiu_kazhu_anchor_mm = 0.0f;
}

static void qiu_tuokun_zhuangtai_qingkong(void)
{
    qiu_kazhu_jiance_qingkong();
    qiu_kazhu_maichong = 0U;
    qiu_kazhu_lengque = 0U;
    qiu_kazhu_maichong_start_ms = 0U;
    qiu_kazhu_lengque_start_ms = 0U;
    qiu_qingjiao_jichu = 0.0f;
}

static void qiu_kazhu_maichong_tingzhi(uint32_t now_ms, uint8_t huifu_jichu)
{
    qiu_kazhu_maichong = 0U;
    qiu_kazhu_lengque = 1U;
    qiu_kazhu_lengque_start_ms = now_ms;
    qiu_kazhu_jiance_qingkong();
    if (huifu_jichu)
    {
        qiu_qingjiao_mubiao = qiu_qingjiao_jichu;
    }
}

void QiuWeizhi_FFReset(void)
{
    qiu_ff_filtered = 0.0f;
    qiu_ff_accel = 0.0f;
    qiu_ff_cmd = 0.0f;
    qiu_guan_ff_cmd = 0.0f;
    qiu_ff_last_updates = bno085_shtp_data.accel_updates;
}

void QiuWeizhi_SetChassisCommandAccelFF(float accel_rpm_s,
                                        float gain_deg_per_rpm_s,
                                        uint8_t enable)
{
    float cmd;

    if (!enable || !qiu_moshi || qiu_chongci_moshi)
    {
        qiu_chassis_cmd_ff = 0.0f;
        return;
    }

    cmd = gain_deg_per_rpm_s * accel_rpm_s;
    qiu_chassis_cmd_ff = PID_LIMIT_MIN_MAX(cmd,
                                           -QIU_CHASSIS_CMD_FF_MAX_DEG,
                                            QIU_CHASSIS_CMD_FF_MAX_DEG);
}

uint8_t QiuWeizhi_GetStuckCount(void)
{
    return qiu_kazhu_count;
}

void QiuWeizhi_SetNegativePulseMode(uint8_t enable)
{
    uint8_t requested = enable ? 1U : 0U;
    uint32_t primask;

    if (qiu_fu_maichong_moshi == requested)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    qiu_fu_maichong_moshi = requested;
    qiu_daoda_baochi = 0U;
    qiu_tuokun_zhuangtai_qingkong();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* 冲刺模式：长距离到点时换可行刹车曲线，并允许近区按球速放开限幅。
 * 只给 H3 用；H5/H6 守中心从来走不到这个工况，保持原行为。 */
void QiuWeizhi_SetDashMode(uint8_t enable)
{
    qiu_chongci_moshi = enable ? 1U : 0U;
}

static void qiu_ff_gengxin(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms;
    float accel;
    float deadband = QIU_FF_DEADBAND;
    float filter_alpha = qiu_ff_physics_enable
                       ? QIU_FF_PHYS_FILTER_ALPHA
                       : QIU_FF_FILTER_ALPHA;
    float ff_max = qiu_absf(qiu_ff_max);

    if (!qiu_ff_enable || qiu_chongci_moshi || !qiu_shijue_xin ||
        bno085_shtp_data.state != SHTP_STATE_RUNNING ||
        bno085_shtp_data.accel_last_ms == 0U)
    {
        QiuWeizhi_FFReset();
        return;
    }

    age_ms = now_ms - bno085_shtp_data.accel_last_ms;
    if (age_ms > QIU_FF_TIMEOUT_MS)
    {
        QiuWeizhi_FFReset();
        return;
    }

    if (bno085_shtp_data.accel_updates != qiu_ff_last_updates)
    {
        qiu_ff_last_updates = bno085_shtp_data.accel_updates;
        /* 实机轴向标定：管道长度方向对应 BNO085 的 Y 轴。 */
        qiu_ff_filtered += filter_alpha *
                           (bno085_shtp_data.accel_y - qiu_ff_filtered);
    }

    accel = qiu_ff_filtered;
    if (qiu_absf(accel) <= deadband)
    {
        accel = 0.0f;
    }
    else if (!qiu_ff_physics_enable && accel > 0.0f)
    {
        accel -= deadband;
    }
    else if (!qiu_ff_physics_enable)
    {
        accel += deadband;
    }

    qiu_ff_accel = accel;
    if (qiu_ff_physics_enable)
    {
        /* 底盘水平加速时沿管方向的平衡条件：
         * g*sin(theta)=a*cos(theta)，所以 theta=atan(a/g)。
         * FK/QIU_FF_PHYSICAL_K 保留原 FK 的小信号增益语义。 */
        qiu_ff_cmd = atan2f(accel, QIU_FF_GRAVITY_M_S2) *
                     QIU_FF_RAD_TO_DEG *
                     (qiu_ff_k / QIU_FF_PHYSICAL_K) *
                     (float)qiu_ff_sign;
    }
    else
    {
        qiu_ff_cmd = qiu_ff_k * accel * (float)qiu_ff_sign;
    }
    qiu_ff_cmd = PID_LIMIT_MIN_MAX(qiu_ff_cmd, -ff_max, ff_max);
}

static void qiu_lishi_qingkong(void)
{
    qiu_lishi_count = 0U;
    qiu_waibu_yiqidong = 0U;
    qiu_waibu_zhijian_frame = 0U;
    qiu_jifen_youxiao = 0U;
    qiu_shang_waibu_ms = 0U;
    qiu_shang_waibu_err = 0.0f;
    qiu_waibu_dt_ms = 0.0f;
    qiu_lishi_kuadu_ms = 0.0f;
    qiu_tuokun_zhuangtai_qingkong();
    qiu_daoda_baochi = 0U;
}

/* 清视觉相关的外环状态，但保留当前实际倾角，让5ms整形平滑回零。 */
static void qiu_waibu_qingkong(void)
{
    PID_Reset(&qiu_pid);
    qiu_lishi_qingkong();
    qiu_su_mm = 0.0f;
    qiu_wucha_mm = 0.0f;
    qiu_mubiao_v = 0.0f;
    qiu_jifen_v = 0.0f;
    qiu_qingjiao_mubiao = 0.0f;
}

void QiuWeizhi_Init(void)
{
    PID_Init(&qiu_pid, QIU_POS_KP_DEFAULT, QIU_POS_KI_DEFAULT, QIU_POS_KD_DEFAULT);
    qiu_vel_kp = QIU_VEL_KP_DEFAULT;
    qiu_vel_kd = QIU_VEL_KD_DEFAULT;
    qiu_kazhu_qingjiao = QIU_KAZHU_QINGJIAO;
    qiu_kazhu_qingjiao_fu = QIU_KAZHU_QINGJIAO;
    QiuWeizhi_Reset();
}

void QiuWeizhi_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    qiu_waibu_qingkong();
    qiu_shang_cmd = 0.0f;
    qiu_qingjiao_cmd = 0.0f;
    qiu_base_cmd = 0.0f;
    qiu_chassis_cmd_ff = 0.0f;
    QiuWeizhi_FFReset();
    qiu_shijue_xin = 0U;
    qiu_rx_yitongbu = 0U;
    qiu_mubiao_yitongbu = 1U;
    qiu_shang_mubiao_mm = qiu_mubiao_mm;
    qiu_waibu_cnt = 0U;
    mubiao_qingjiao = 0.0f;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void qiu_jifen_xianfu(void)
{
    float shang;
    float vmax = QIU_MUBIAO_V_MAX;

    if (qiu_pid.Ki > 1e-6f || qiu_pid.Ki < -1e-6f)
    {
        shang = (0.20f * vmax) / qiu_absf(qiu_pid.Ki);
        qiu_pid.err_sum = PID_LIMIT_MIN_MAX(qiu_pid.err_sum, -shang, shang);
    }
    else
    {
        qiu_pid.err_sum = 0.0f;
    }
}

static void qiu_diushi_chuli(void)
{
    if (qiu_shijue_xin)
    {
        qiu_diushi_cnt++;
    }
    qiu_shijue_xin = 0U;
    qiu_rx_yitongbu = 0U;
    qiu_waibu_qingkong();
}

/* 软死区只压低中心噪声，P项始终使用最新一帧误差。 */
static float qiu_ruan_siqu(float err, float siqu)
{
    float a;

    if (siqu <= 1e-3f)
    {
        return err;
    }
    a = qiu_absf(err);
    if (a >= siqu)
    {
        return err;
    }
    return err * (a / siqu);
}

static float qiu_v_xian_by_err(float abs_err)
{
    float vmax = QIU_MUBIAO_V_MAX;
    float local;

    if (qiu_chongci_moshi)
    {
        /* 可行刹车曲线 v ≤ √(2a·err)：K=34 时 18mm 对应 144mm/s，
         * 正好是近区限幅刹得住的入弯速度。旧分段曲线在同一处允许
         * 264mm/s，冲过头约40mm，再被推回来→来回晃。 */
        local = qiu_shache_k * sqrtf(abs_err);
        if (local > vmax)
        {
            local = vmax;
        }
        if (local < QIU_V_XIAN_DI)
        {
            local = QIU_V_XIAN_DI;
        }
        return local;
    }

    if (abs_err >= 40.0f)
    {
        return vmax;
    }
    if (abs_err >= 18.0f)
    {
        local = vmax * (0.55f + 0.45f * ((abs_err - 18.0f) / 22.0f));
        return local;
    }

    local = 60.0f + 12.0f * abs_err;
    if (local > vmax * 0.55f)
    {
        local = vmax * 0.55f;
    }
    if (local < 50.0f)
    {
        local = 50.0f;
    }
    return local;
}

static void qiu_lishi_jiaru(float position_mm, uint32_t timestamp_ms)
{
    uint8_t i;

    if (qiu_lishi_count < QIU_LISHI_YANGBEN)
    {
        qiu_lishi[qiu_lishi_count].position_mm = position_mm;
        qiu_lishi[qiu_lishi_count].timestamp_ms = timestamp_ms;
        qiu_lishi_count++;
        return;
    }

    for (i = 1U; i < QIU_LISHI_YANGBEN; i++)
    {
        qiu_lishi[i - 1U] = qiu_lishi[i];
    }
    qiu_lishi[QIU_LISHI_YANGBEN - 1U].position_mm = position_mm;
    qiu_lishi[QIU_LISHI_YANGBEN - 1U].timestamp_ms = timestamp_ms;
}

/* 4点最小二乘斜率，比只取两个端点更不容易被0.8mm量化台阶带偏。 */
static uint8_t qiu_lishi_sulv(float *velocity_mm_s, uint32_t *newest_ms)
{
    float t[QIU_LISHI_YANGBEN];
    float t_mean = 0.0f;
    float x_mean = 0.0f;
    float fenzi = 0.0f;
    float fenmu = 0.0f;
    uint32_t first_ms;
    uint32_t span_ms;
    uint8_t i;

    if (qiu_lishi_count < QIU_LISHI_YANGBEN)
    {
        return 0U;
    }

    first_ms = qiu_lishi[0].timestamp_ms;
    *newest_ms = qiu_lishi[QIU_LISHI_YANGBEN - 1U].timestamp_ms;
    span_ms = *newest_ms - first_ms;
    qiu_lishi_kuadu_ms = (float)span_ms;
    if (span_ms == 0U || span_ms > QIU_LISHI_MAX_KUADU_MS)
    {
        return 0U;
    }

    for (i = 0U; i < QIU_LISHI_YANGBEN; i++)
    {
        t[i] = (float)(qiu_lishi[i].timestamp_ms - first_ms) * 0.001f;
        t_mean += t[i];
        x_mean += qiu_lishi[i].position_mm;
    }
    t_mean /= (float)QIU_LISHI_YANGBEN;
    x_mean /= (float)QIU_LISHI_YANGBEN;

    for (i = 0U; i < QIU_LISHI_YANGBEN; i++)
    {
        float dt = t[i] - t_mean;
        fenzi += dt * (qiu_lishi[i].position_mm - x_mean);
        fenmu += dt * dt;
    }
    if (fenmu < 1e-7f)
    {
        return 0U;
    }

    *velocity_mm_s = fenzi / fenmu;
    return 1U;
}

static uint8_t qiu_waibu_daoshi(void)
{
    if (qiu_lishi_count < QIU_LISHI_YANGBEN)
    {
        return 0U;
    }
    if (!qiu_waibu_yiqidong)
    {
        qiu_waibu_yiqidong = 1U;
        qiu_waibu_zhijian_frame = 0U;
        return 1U;
    }

    qiu_waibu_zhijian_frame++;
    if (qiu_waibu_zhijian_frame >= QIU_WAIBU_FRAME_FENPIN)
    {
        qiu_waibu_zhijian_frame = 0U;
        return 1U;
    }
    return 0U;
}

static void qiu_weizhi_huan(float err_yuan, float velocity_mm_s,
                            float dt_s, uint8_t has_dt)
{
    float err = qiu_ruan_siqu(err_yuan, qiu_siqu_mm);
    float abs_err = qiu_absf(err_yuan);
    float out_v;
    float vmax;
    float kd = qiu_pid.Kd;

    if (has_dt && qiu_jifen_youxiao)
    {
        if (abs_err < QIU_JIN_MM)
        {
            qiu_pid.err_sum += 0.5f * (err + qiu_shang_waibu_err) *
                               (dt_s / QIU_JIFEN_JIZHUN_S);
        }
        else
        {
            float shuai = 1.0f - 8.0f * dt_s;
            if (shuai < 0.0f)
            {
                shuai = 0.0f;
            }
            qiu_pid.err_sum *= shuai;
        }
    }
    qiu_jifen_youxiao = 1U;
    qiu_shang_waibu_err = err;
    qiu_jifen_xianfu();

    /* 普通回中一旦进入验收死区就卸掉历史积分，避免球已经到中心，
     * 积分仍维持同方向目标速度把球继续推到另一侧。H3 保持原行为。 */
    if (qiu_zhongxin_huizhong_moshi() && abs_err <= qiu_siqu_mm)
    {
        qiu_pid.err_sum = 0.0f;
    }
    qiu_jifen_v = qiu_pid.Ki * qiu_pid.err_sum;

    /* 只给普通回中视觉负侧的向心运动增加阻尼；H3和反向恢复不变。 */
    if (qiu_zhongxin_huizhong_moshi() && err_yuan > 0.0f &&
        velocity_mm_s > 0.0f)
    {
        kd += QIU_ZHONGXIN_NEG_SIDE_KD_ADD;
    }

    /* 目标不变时 d(error)/dt=-qvel；对测量微分不会产生目标阶跃冲击。 */
    out_v = qiu_pid.Kp * err
          + qiu_jifen_v
          - kd * velocity_mm_s;

    vmax = qiu_v_xian_by_err(abs_err);
    out_v = PID_LIMIT_MIN_MAX(out_v, -vmax, vmax);
    qiu_mubiao_v = out_v;

    qiu_pid.target_val = qiu_mubiao_mm;
    qiu_pid.actual_val = qiu_mubiao_mm - err_yuan;
    qiu_pid.err        = err;
    qiu_pid.err_last   = err_yuan;
    qiu_pid.output     = out_v;
}

static void qiu_qingjiao_jisuan(float shice, float err_yuan, float velocity_mm_s)
{
    uint32_t now_ms = HAL_GetTick();
    float abs_err = qiu_absf(err_yuan);
    float abs_v = qiu_absf(velocity_mm_s);
    float c_yong = qiu_vel_kp;
    float o_yong = qiu_vel_kd;
    float vel_err;
    float out;
    float hmax;
    float xian;
    float kazhu_err_menxian = QIU_KAZHU_ERR_MM;
    uint8_t zhongxin_huizhong = qiu_zhongxin_huizhong_moshi();
    uint8_t lengque_zhong;

    if (zhongxin_huizhong)
    {
        kazhu_err_menxian = qiu_siqu_mm + 0.4f;
        if (kazhu_err_menxian < QIU_ZHONGXIN_KAZHU_ERR_MIN_MM)
        {
            kazhu_err_menxian = QIU_ZHONGXIN_KAZHU_ERR_MIN_MM;
        }
        if (kazhu_err_menxian > QIU_KAZHU_ERR_MM)
        {
            kazhu_err_menxian = QIU_KAZHU_ERR_MM;
        }
    }

    if (zhongxin_huizhong && err_yuan > 0.0f && velocity_mm_s > 0.0f)
    {
        o_yong += QIU_ZHONGXIN_NEG_SIDE_VEL_KD_ADD;
    }

    if (abs_err < QIU_JIN_MM)
    {
        float t = abs_err / QIU_JIN_MM;
        c_yong = qiu_vel_kp * (0.70f + 0.30f * t);
        o_yong = qiu_vel_kd * (1.55f - 0.55f * t);
    }

    vel_err = qiu_mubiao_v - velocity_mm_s;
    out = c_yong * vel_err - o_yong * velocity_mm_s;
    out *= (float)qiu_fuhao;

    hmax = qiu_absf(qiu_qingjiao_max);
    xian = hmax;
    if (abs_err < QIU_JIN_MM)
    {
        float t = abs_err / QIU_JIN_MM;
        float jin_xian = QIU_JIN_H_MIN + (QIU_JIN_H_MAX - QIU_JIN_H_MIN) * t;

        /* 冲刺模式：球还快的时候不能剥夺刹车权限，按球速把上限拉回
         * hmax；球慢下来自动退回调好的安静限幅，中心手感不变。 */
        if (qiu_chongci_moshi && abs_v > 1.0f)
        {
            float k = abs_v / QIU_JIN_SU_CANKAO;
            if (k > 1.0f)
            {
                k = 1.0f;
            }
            jin_xian += (hmax - jin_xian) * k;
        }
        if (jin_xian < xian)
        {
            xian = jin_xian;
        }
    }
    if (xian < 1.0f)
    {
        xian = 1.0f;
    }
    out = PID_LIMIT_MIN_MAX(out, -xian, xian);

    /* 视觉负侧向心运动的最后一段保证最小刹车角，避免低速越过零点后
     * 被正侧自然凹点继续拉走。只给普通目标零点回中使用。 */
    if (zhongxin_huizhong && err_yuan > 0.0f &&
        abs_err <= QIU_ZHONGXIN_NEG_BRAKE_ZONE_MM &&
        velocity_mm_s >= QIU_ZHONGXIN_NEG_BRAKE_SPEED_MM_S)
    {
        float brake = -QIU_ZHONGXIN_NEG_BRAKE_DEG * (float)qiu_fuhao;
        if (out * (float)qiu_fuhao > -QIU_ZHONGXIN_NEG_BRAKE_DEG)
        {
            out = brake;
        }
        out = PID_LIMIT_MIN_MAX(out, -xian, xian);
    }

    /* 球相对当前目标继续跑远时才启动边界护栏。目标为0时与原逻辑等价，
     * 非零目标则不会把朝目标运动误判成朝管壁外跑。 */
    if (QIU_BIANJIE_QINGJIAO > 1e-3f && abs_err >= (QIU_BIANJIE_MM * 0.90f))
    {
        uint8_t wai_pao = 0U;
        if ((err_yuan > 0.0f && velocity_mm_s < -10.0f) ||
            (err_yuan < 0.0f && velocity_mm_s > 10.0f))
        {
            wai_pao = 1U;
        }
        if (wai_pao)
        {
            float guard = QIU_BIANJIE_QINGJIAO;
            if (abs_err > QIU_BIANJIE_MM)
            {
                float k = abs_err / QIU_BIANJIE_MM;
                if (k > 1.6f)
                {
                    k = 1.6f;
                }
                guard *= k;
            }
            if (err_yuan < 0.0f)
            {
                guard = -guard * (float)qiu_fuhao;
                if (out > guard)
                {
                    out = guard;
                }
            }
            else
            {
                guard = guard * (float)qiu_fuhao;
                if (out < guard)
                {
                    out = guard;
                }
            }
            out = PID_LIMIT_MIN_MAX(out, -hmax, hmax);
        }
    }

    if (qiu_kazhu_lengque &&
        (now_ms - qiu_kazhu_lengque_start_ms) >= qiu_kazhu_lengque_ms)
    {
        qiu_kazhu_lengque = 0U;
    }

    /* 基础闭环始终保留。脱困脉冲只临时覆盖它，脉冲结束后立即恢复，
     * 不允许在目标窗口边界切换成平台回水平。 */
    qiu_qingjiao_jichu = out;

    /* 负向脉冲在球已经朝目标运动或达到最大宽度时立即撤销。 */
    if (qiu_kazhu_maichong &&
        ((now_ms - qiu_kazhu_maichong_start_ms) >= qiu_kazhu_maichong_ms ||
         velocity_mm_s <= -QIU_KAZHU_TUOLI_SU_MM_S || err_yuan >= 0.0f))
    {
        qiu_kazhu_maichong_tingzhi(now_ms, 0U);
    }

    /* 偏离目标后若连续约200ms位移不足1mm，普通位置增益可能小于
     * 静摩擦阈值。脉冲期间不累计；冷却期间照常累计确认，只是到点了
     * 先不发脉冲——否则 100ms脉冲+200ms冷却+6拍重新确认(≈220ms)
     * 让负向占空比只剩约19%，静摩擦大的一侧5s内爬不到位。 */
    lengque_zhong = (qiu_fu_maichong_moshi && err_yuan < 0.0f &&
                     qiu_kazhu_lengque);

    /* 普通回中经过正侧自然凹点时，低速即提前启用原脱困角。
     * H3 的 dash 模式不会进入 zhongxin_huizhong。 */
    if (zhongxin_huizhong && err_yuan < 0.0f &&
        abs_err >= kazhu_err_menxian &&
        abs_err <= QIU_ZHONGXIN_POS_YUTUOKUN_ERR_MAX_MM &&
        abs_v <= QIU_ZHONGXIN_POS_YUTUOKUN_SU_MM_S &&
        !qiu_kazhu_maichong)
    {
        qiu_kazhu_anchor_mm = shice;
        qiu_kazhu_anchor_valid = 1U;
        qiu_kazhu_count = QIU_KAZHU_WAIBU_CI;
    }

    if (!qiu_kazhu_maichong && abs_err >= kazhu_err_menxian)
    {
        if (!qiu_kazhu_anchor_valid ||
            qiu_absf(shice - qiu_kazhu_anchor_mm) > QIU_KAZHU_WEIYI_MM)
        {
            qiu_kazhu_anchor_mm = shice;
            qiu_kazhu_anchor_valid = 1U;
            qiu_kazhu_count = 1U;
        }
        else if (qiu_kazhu_count < QIU_KAZHU_WAIBU_CI)
        {
            qiu_kazhu_count++;
        }
    }
    else if (!qiu_kazhu_maichong)
    {
        qiu_kazhu_jiance_qingkong();
    }
    if (qiu_kazhu_count >= QIU_KAZHU_WAIBU_CI && !lengque_zhong)
    {
        float tuokun_k = (err_yuan < 0.0f)
                       ? qiu_kazhu_qingjiao_fu : qiu_kazhu_qingjiao;
        float tuokun_abs = qiu_absf(tuokun_k);
        float tuokun;

        /* 普通回中正侧静摩擦较大：只提高负向倾角的脱困峰值。
         * H3 的 dash 模式不会进入 zhongxin_huizhong，原行为不变。 */
        if (zhongxin_huizhong && err_yuan < 0.0f)
        {
            tuokun_abs = QIU_ZHONGXIN_KAZHU_POS_SIDE_DEG;
        }

        /* 只给普通中心回中缩小近目标脱困倾角：2.4mm 附近从 1.2deg
         * 起步，误差到 4mm 线性恢复原值。球一移动 1mm 检测即撤销。 */
        if (zhongxin_huizhong && kazhu_err_menxian < QIU_KAZHU_ERR_MM &&
            abs_err < QIU_KAZHU_ERR_MM)
        {
            float k = (abs_err - kazhu_err_menxian) /
                      (QIU_KAZHU_ERR_MM - kazhu_err_menxian);
            float min_tilt = QIU_ZHONGXIN_KAZHU_TILT_MIN_DEG;
            if (k < 0.0f)
            {
                k = 0.0f;
            }
            if (k > 1.0f)
            {
                k = 1.0f;
            }
            if (min_tilt > tuokun_abs)
            {
                min_tilt = tuokun_abs;
            }
            tuokun_abs = min_tilt + (tuokun_abs - min_tilt) * k;
        }
        tuokun = tuokun_abs * (float)qiu_fuhao;
        if (err_yuan < 0.0f)
        {
            tuokun = -tuokun;
        }
        /* 只在脉冲还没起来时打时间戳。脉冲期间卡滞计数被冻结在阈值上，
         * 这里若每拍重盖 start_ms，5ms整形环永远等不到100ms宽度，
         * 脉冲就变成常开的6deg，粘滑往返照旧。 */
        if (qiu_fu_maichong_moshi && err_yuan < 0.0f && !qiu_kazhu_maichong)
        {
            qiu_kazhu_maichong = 1U;
            qiu_kazhu_maichong_start_ms = now_ms;
            qiu_kazhu_lengque = 0U;
        }
        if ((!qiu_fu_maichong_moshi || err_yuan >= 0.0f ||
             qiu_kazhu_maichong) &&
            ((tuokun > 0.0f && out < tuokun) ||
             (tuokun < 0.0f && out > tuokun)))
        {
            out = tuokun;
        }
        /* 脱困脉冲用于克服已确认的静摩擦，可越过近目标角度整形上限；
         * 仍受平台总倾角限制，且位移超过1mm后卡滞计数会立即清零。 */
        out = PID_LIMIT_MIN_MAX(out, -hmax, hmax);
    }

    if (abs_err < 0.8f && abs_v < 10.0f && out < 0.25f && out > -0.25f)
    {
        out = 0.0f;
    }
    qiu_qingjiao_mubiao = out;
    qiu_base_cmd = out;
}

static void qiu_qingjiao_zhengxing(void)
{
    uint32_t now_ms = HAL_GetTick();
    float abs_err = qiu_absf(qiu_wucha_mm);
    float sulv = qiu_sulv_max;
    float bu;
    float hmax;
    float mubiao;

    qiu_ff_gengxin();

    qiu_daoda_baochi = 0U;
    if (qiu_kazhu_maichong &&
        (now_ms - qiu_kazhu_maichong_start_ms) >= qiu_kazhu_maichong_ms)
    {
        qiu_kazhu_maichong_tingzhi(now_ms, 1U);
    }

    hmax = qiu_absf(qiu_qingjiao_max);
    if (qiu_shijue_xin && qiu_moshi)
    {
        qiu_guan_ff_cmd = qiu_guan_ff_k * qiu_mubiao_mm;
        qiu_guan_ff_cmd = PID_LIMIT_MIN_MAX(qiu_guan_ff_cmd,
                                            -QIU_GUAN_FF_MAX_DEFAULT,
                                            QIU_GUAN_FF_MAX_DEFAULT);
    }
    else
    {
        qiu_guan_ff_cmd = 0.0f;
    }
    /* 物理前馈也必须经过平台倾角变化率保护。IMU安装在底盘上，若将
     * 电机目标完全直通，执行器反作用会被IMU再次测到并形成机械正反馈。 */
    mubiao = qiu_qingjiao_mubiao + qiu_guan_ff_cmd +
             qiu_ff_cmd + qiu_chassis_cmd_ff;
    mubiao = PID_LIMIT_MIN_MAX(mubiao, -hmax, hmax);

    if (qiu_shijue_xin && abs_err < QIU_JIN_MM)
    {
        sulv *= 0.75f;
    }
    bu = sulv * QIU_SERVICE_ZHOUQI_S;
    if (bu > 0.0f)
    {
        if (mubiao > qiu_shang_cmd + bu)
        {
            qiu_shang_cmd += bu;
        }
        else if (mubiao < qiu_shang_cmd - bu)
        {
            qiu_shang_cmd -= bu;
        }
        else
        {
            qiu_shang_cmd = mubiao;
        }
    }
    else
    {
        qiu_shang_cmd = mubiao;
    }

    qiu_shang_cmd = PID_LIMIT_MIN_MAX(qiu_shang_cmd, -hmax, hmax);
    qiu_qingjiao_cmd = qiu_shang_cmd;
    mubiao_qingjiao = qiu_qingjiao_cmd;
    jiaodu_moshi = 1U;
}

static void qiu_waibu_jisuan(void)
{
    float velocity_mm_s;
    float err_yuan;
    float dt_s = 0.0f;
    uint32_t newest_ms;
    uint32_t dt_ms = 0U;
    uint8_t has_dt = 0U;

    if (!qiu_lishi_sulv(&velocity_mm_s, &newest_ms))
    {
        qiu_waibu_qingkong();
        return;
    }

    /* 不做速度死区：清零会让速度环在最后几毫米算出反号倾角
     * （球以10mm/s回中、命令3mm/s时，真实vel_err=-7该刹车，
     *   清零后变+3去加速），而且一并丢掉阻尼项。
     * 4点最小二乘跨约55ms本身就是滤波；0.8mm端点台阶的
     * 保守速度峰值约13.3mm/s，已远小于旧算法的80mm/s。 */
    qiu_su_mm = velocity_mm_s;

    if (qiu_shang_waibu_ms != 0U)
    {
        dt_ms = newest_ms - qiu_shang_waibu_ms;
        if (dt_ms > 0U && dt_ms <= QIU_SHIJUE_GUOQI_MS)
        {
            dt_s = (float)dt_ms * 0.001f;
            has_dt = 1U;
        }
        else
        {
            qiu_pid.err_sum = 0.0f;
            qiu_jifen_youxiao = 0U;
        }
    }
    qiu_shang_waibu_ms = newest_ms;
    qiu_waibu_dt_ms = (float)dt_ms;

    err_yuan = qiu_mubiao_mm - qiu_lishi[QIU_LISHI_YANGBEN - 1U].position_mm;
    qiu_wucha_mm = err_yuan;
    qiu_weizhi_huan(err_yuan, velocity_mm_s, dt_s, has_dt);
    qiu_qingjiao_jisuan(qiu_lishi[QIU_LISHI_YANGBEN - 1U].position_mm,
                        err_yuan, velocity_mm_s);
    qiu_waibu_cnt++;
}

void QiuWeizhi_Update(void)
{
    Vision_Snapshot_t snapshot;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms;

    if (!qiu_moshi)
    {
        return;
    }
    if (!Vision_GetSnapshot(&snapshot))
    {
        qiu_diushi_chuli();
        qiu_qingjiao_zhengxing();
        return;
    }

    if (!qiu_mubiao_yitongbu ||
        qiu_absf(qiu_mubiao_mm - qiu_shang_mubiao_mm) > QIU_MUBIAO_BIANHUA_MM)
    {
        qiu_shang_mubiao_mm = qiu_mubiao_mm;
        qiu_mubiao_yitongbu = 1U;
        qiu_waibu_qingkong();
    }

    age_ms = now_ms - snapshot.last_rx_ms;
    if (!snapshot.valid || !snapshot.connected || snapshot.rx_count == 0U ||
        age_ms > QIU_SHIJUE_GUOQI_MS)
    {
        qiu_diushi_chuli();
        qiu_qingjiao_zhengxing();
        return;
    }
    qiu_shijue_xin = 1U;

    if (!qiu_rx_yitongbu)
    {
        qiu_shang_rx_count = snapshot.rx_count;
        qiu_rx_yitongbu = 1U;
    }
    else if (snapshot.rx_count != qiu_shang_rx_count)
    {
        uint32_t rx_delta = snapshot.rx_count - qiu_shang_rx_count;
        qiu_shang_rx_count = snapshot.rx_count;
        if (rx_delta > 1U)
        {
            qiu_waibu_qingkong();
        }
    }
    else
    {
        qiu_qingjiao_zhengxing();
        return;
    }

    qiu_wucha_mm = qiu_mubiao_mm - snapshot.distance_mm;
    qiu_lishi_jiaru(snapshot.distance_mm, snapshot.last_rx_ms);
    if (qiu_waibu_daoshi())
    {
        qiu_waibu_jisuan();
    }
    qiu_qingjiao_zhengxing();
}
