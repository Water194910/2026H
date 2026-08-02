#ifndef __QIU_WEIZHI_H
#define __QIU_WEIZHI_H

#include <stdint.h>
#include "pid.h"

/* ── 球位置：视觉约55fps/2 → 球速环 → 5ms倾角整形 ───────────
 * 目标手感：一步到位 + 近区精确刹停（不要缓-停-缓-停）
 * 远区：大倾角/高变化率/高目标球速
 * 近区：靠 Z/O 阻尼钉住，而不是把 H/M 砍碎
 * 串口 X/Y/Z 位置；C/O 球速；G/H/U/M/N/B
 * ─────────────────────────────────────────────────────────── */

#define QIU_SERVICE_ZHOUQI_S       0.005f
#define QIU_DAYIN_FENPIN          20U  /* 10Hz：保留动态观测并减少串口刷屏 */
#define QIU_WAIBU_FRAME_FENPIN     2U
#define QIU_LISHI_YANGBEN          4U
#define QIU_JIFEN_JIZHUN_S         0.010f
#define QIU_SHIJUE_GUOQI_MS        60U
#define QIU_LISHI_MAX_KUADU_MS     120U
#define QIU_MUBIAO_BIANHUA_MM      0.01f

/* DT0 实机参数。Ki 已由20ms的0.005等效换算到10ms。 */
#define QIU_POS_KP_DEFAULT    3.00f
#define QIU_POS_KI_DEFAULT    0.0025f
#define QIU_POS_KD_DEFAULT    0.0f
#define QIU_MUBIAO_V_MAX      480.0f

#define QIU_KP_DEFAULT        QIU_POS_KP_DEFAULT
#define QIU_KI_DEFAULT        QIU_POS_KI_DEFAULT
#define QIU_KD_DEFAULT        QIU_POS_KD_DEFAULT

/* 球速环：小比例配合速度阻尼，兼顾回中和中心静稳。 */
#define QIU_VEL_KP_DEFAULT    0.032f
#define QIU_VEL_KD_DEFAULT    0.010f

#define QIU_MUBIAO_MM         0.0f
#define QIU_QINGJIAO_MAX      JIAODU_GONGZUO_DU  /* 默认与角度环工作半宽一致 */
#define QIU_SIQU_MM           1.0f
#define QIU_SULV_MAX          320.0f   /* °/s，别再 90 那种肉 */

/* 近区只做轻度收窄，别把权威砍没 */
#define QIU_JIN_MM            18.0f
#define QIU_JIN_H_MAX         8.0f
#define QIU_JIN_H_MIN         2.5f

/* 越界护栏：以10mm比赛边界为基准，从90%位置开始预介入。 */
#define QIU_BIANJIE_MM        10.0f
#define QIU_BIANJIE_QINGJIAO  3.5f

/* 静摩擦脱困：连续低速且偏离中心时给最小倾角，球一动立即撤销。 */
#define QIU_KAZHU_ERR_MM       4.0f
#define QIU_KAZHU_WEIYI_MM     1.0f
#define QIU_KAZHU_WAIBU_CI     6U
#define QIU_KAZHU_QINGJIAO     2.2f

/* 普通回中专用：验收线外不能留下 2~4mm 的静摩擦空档。
 * H3 冲刺仍使用上面的 4mm 阈值和原脱困倾角。 */
#define QIU_ZHONGXIN_KAZHU_ERR_MIN_MM   2.4f
#define QIU_ZHONGXIN_KAZHU_TILT_MIN_DEG 1.2f
/* 实测球在正侧 +8~10mm 时，原 -2.2deg 无法克服静摩擦。
 * 仅普通回中在该方向提高脱困角；H3 继续使用自己的 2.2/6deg。 */
#define QIU_ZHONGXIN_KAZHU_POS_SIDE_DEG 3.4f
/* 正侧存在自然凹点。球已低速时提前触发现有脱困角，避免完全停住
 * 再等约200ms；近中心仍由2.4~4mm的倾角渐变限制冲量。 */
#define QIU_ZHONGXIN_POS_YUTUOKUN_ERR_MAX_MM  14.0f
#define QIU_ZHONGXIN_POS_YUTUOKUN_SU_MM_S     12.0f
/* 管道两侧坡度不对称：视觉负侧向中心运动时单独增加外环速度阻尼。 */
#define QIU_ZHONGXIN_NEG_SIDE_KD_ADD            0.24f
/* 同一方向近中心仍需更多直接球速刹车，防止低速越过零点后滑入凹点。 */
#define QIU_ZHONGXIN_NEG_SIDE_VEL_KD_ADD         0.030f
/* 视觉负侧接近中心时保证最小反向刹车，避免越过零点后滑入正侧凹点。 */
#define QIU_ZHONGXIN_NEG_BRAKE_ZONE_MM            8.0f
#define QIU_ZHONGXIN_NEG_BRAKE_SPEED_MM_S         8.0f
#define QIU_ZHONGXIN_NEG_BRAKE_DEG                2.0f

/* H3负向末段专用：大静摩擦倾角只给短脉冲，防止持续6deg形成粘滑往返。
 * 宽度/冷却做成运行时可调（QW/QC），负向爬不到位时先动这两个。 */
#define QIU_KAZHU_MAICHONG_MS       100U
#define QIU_KAZHU_LENGQUE_MS        200U
#define QIU_KAZHU_TUOLI_SU_MM_S      14.0f

/* H3长距离到点（冲刺模式）：
 * 近区倾角上限(2.5°~8°)能提供的刹车加速度只有 0.30~0.97m/s²，
 * 积出来最多吃掉≈147mm/s的入弯速度；而旧分段速度曲线在18mm处
 * 允许264mm/s，物理上必然冲过头约40mm，跟X/Z/C/O怎么调都无关。
 * 所以冲刺模式改用可行刹车曲线 v ≤ K·√err（K=√(2a)，a≈0.6m/s²→34），
 * 并在球还快的时候按球速把近区上限拉回 hmax，把刹车权限还给它。 */
#define QIU_SHACHE_K_DEFAULT         34.0f   /* (mm/s)/√mm */
#define QIU_V_XIAN_DI                50.0f   /* mm/s 下限，防近区推不动 */
#define QIU_JIN_SU_CANKAO            90.0f   /* mm/s，到此速度放开近区限幅 */

/* 车体加速度前馈：先用部分物理补偿，实测后再逐步逼近1/g。 */
#define QIU_FF_ENABLE_DEFAULT          1U
#define QIU_FF_PHYSICS_ENABLE_DEFAULT  1U
#define QIU_FF_K_DEFAULT        5.84f  /* deg/(m/s^2) */
#define QIU_FF_MAX_DEFAULT      14.0f  /* deg */
#define QIU_FF_SIGN_DEFAULT            1
#define QIU_FF_DEADBAND         0.08f  /* m/s^2 */
#define QIU_FF_FILTER_ALPHA     0.50f
#define QIU_FF_PHYS_FILTER_ALPHA 0.90f
#define QIU_FF_TIMEOUT_MS       25U
#define QIU_FF_GRAVITY_M_S2     9.80665f
#define QIU_FF_RAD_TO_DEG       57.2957795f
/* atan(a/g) 在零点附近的斜率，用它归一化后 FK 仍保持 deg/(m/s^2) 语义。 */
#define QIU_FF_PHYSICAL_K       (QIU_FF_RAD_TO_DEG / QIU_FF_GRAVITY_M_S2)

/* H4/H5/H6底盘目标加速度前馈：增益由比赛任务提供，H4最高5.5deg。 */
#define QIU_CHASSIS_CMD_FF_MAX_DEG           5.5f

/* 管道曲率前馈：实测在 +/-50mm 处分别需要 +/-2deg 才能静态平衡。 */
#define QIU_GUAN_FF_K_DEFAULT    0.040f  /* deg/mm */
#define QIU_GUAN_FF_MAX_DEFAULT  4.0f    /* deg */

extern PID_t qiu_pid;

extern volatile uint8_t  qiu_moshi;
extern volatile float    qiu_mubiao_mm;
extern volatile float    qiu_qingjiao_max;
extern volatile float    qiu_siqu_mm;
extern volatile float    qiu_sulv_max;
extern volatile int8_t   qiu_fuhao;

extern volatile float    qiu_vel_kp;
extern volatile float    qiu_vel_kd;
extern volatile float    qiu_kazhu_qingjiao;
extern volatile float    qiu_kazhu_qingjiao_fu;

extern volatile float    qiu_wucha_mm;
extern volatile float    qiu_su_mm;
extern volatile float    qiu_mubiao_v;
extern volatile float    qiu_jifen_v;
extern volatile float    qiu_qingjiao_cmd;
extern volatile float    qiu_base_cmd;
extern volatile uint8_t  qiu_ff_enable;
extern volatile uint8_t  qiu_ff_physics_enable;
extern volatile float    qiu_ff_k;
extern volatile float    qiu_ff_max;
extern volatile int8_t   qiu_ff_sign;
extern volatile float    qiu_ff_accel;
extern volatile float    qiu_ff_cmd;
extern volatile float    qiu_chassis_cmd_ff;
extern volatile float    qiu_guan_ff_k;
extern volatile float    qiu_guan_ff_cmd;
extern volatile uint32_t qiu_diushi_cnt;
extern volatile uint32_t qiu_waibu_cnt;
extern volatile float    qiu_waibu_dt_ms;
extern volatile float    qiu_lishi_kuadu_ms;
extern volatile uint8_t  qiu_kazhu_maichong;
extern volatile uint8_t  qiu_daoda_baochi;

extern volatile uint8_t  qiu_chongci_moshi;
extern volatile float    qiu_shache_k;
extern volatile uint32_t qiu_kazhu_maichong_ms;
extern volatile uint32_t qiu_kazhu_lengque_ms;

void QiuWeizhi_Init(void);
void QiuWeizhi_Reset(void);
void QiuWeizhi_Update(void);
void QiuWeizhi_FFReset(void);
void QiuWeizhi_SetChassisCommandAccelFF(float accel_rpm_s,
                                        float gain_deg_per_rpm_s,
                                        uint8_t enable);
uint8_t QiuWeizhi_GetStuckCount(void);
void QiuWeizhi_SetNegativePulseMode(uint8_t enable);
void QiuWeizhi_SetDashMode(uint8_t enable);

#endif
