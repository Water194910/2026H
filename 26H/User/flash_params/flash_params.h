#ifndef __FLASH_PARAMS_H
#define __FLASH_PARAMS_H

#include <stdint.h>
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 26h Flash 参数（从 ytbeif 完整流程移植，结构为本工程专用）────
 *
 * 地址：Sector7 @ 0x08060000（与 ytbeif 同扇区，magic 不同不混读）
 * 流程：上电 FlashParams_Load → 调参 → 串口 SAVE / W 写入
 * 校验：magic + version + CRC32
 * ─────────────────────────────────────────────────────────── */

#define FLASH_SAVE_ADDR       0x08060000U
#define FLASH_SAVE_MAGIC      0x48363236U   /* "H626" = 26h */
#define FLASH_SAVE_VERSION    8U            /* v8：加入 IMU 前馈参数 */

/* v2 旧结构（只用于读旧存档并迁移，不再写入）。
 * 不这么做的话，今晚调好存进去的速度/角度参数会被当成"版本不认"丢掉 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float    spd_kp, spd_ki, spd_kd;
    float    ang_kp, ang_ki, ang_kd;
    int32_t  pingheng_raw;
    float    gongzuo_du;
    int8_t   zhuanxiang;
    uint8_t  reserved[3];
    uint32_t crc;
} FlashParamsV2_t;

/* v5：串级前/球速环未入库的存档布局（CRC 按此长度校验） */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float spd_kp, spd_ki, spd_kd;
    float ang_kp, ang_ki, ang_kd;
    int32_t  pingheng_raw;
    float    gongzuo_du;
    int8_t   zhuanxiang;
    uint8_t  reserved[3];
    float    qiu_kp, qiu_ki, qiu_kd;
    float    qiu_mubiao_mm;
    float    qiu_qingjiao_max;
    float    qiu_siqu_mm;
    float    qiu_sulv_max;
    int8_t   qiu_fuhao;
    uint8_t  reserved2[3];
    uint32_t crc;
} FlashParamsV5_t;

/* v6/v7：仅用于校验并迁移旧存档，新固件不再按此布局写入。 */
typedef struct
{
    uint32_t magic;
    uint32_t version;

    /* 速度环 2ms */
    float spd_kp;
    float spd_ki;
    float spd_kd;

    /* 角度环 5ms */
    float ang_kp;
    float ang_ki;
    float ang_kd;

    /* 标定 */
    int32_t  pingheng_raw;   /* 水平零点 raw */
    float    gongzuo_du;     /* 工作半宽 ° */
    int8_t   zhuanxiang;     /* +1 / -1 */
    uint8_t  reserved[3];

    /* v3-v7：球位置/球速环；v6 起含 vel_kp/vel_kd */
    float    qiu_kp;
    float    qiu_ki;
    float    qiu_kd;
    float    qiu_mubiao_mm;    /* 目标球位置 mm */
    float    qiu_qingjiao_max; /* 输出倾角限幅 ° */
    float    qiu_siqu_mm;      /* 位置死区 mm */
    float    qiu_sulv_max;     /* 倾角变化率 °/s */
    int8_t   qiu_fuhao;        /* 推球方向符号 +1/-1 */
    uint8_t  reserved2[3];
    float    qiu_vel_kp;       /* v6+：球速环 Kp °/(mm/s) */
    float    qiu_vel_kd;       /* v6+：球速环阻尼 */

    uint32_t crc;
} FlashParamsV7_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;

    float spd_kp;
    float spd_ki;
    float spd_kd;
    float ang_kp;
    float ang_ki;
    float ang_kd;

    int32_t  pingheng_raw;
    float    gongzuo_du;
    int8_t   zhuanxiang;
    uint8_t  reserved[3];

    float    qiu_kp;
    float    qiu_ki;
    float    qiu_kd;
    float    qiu_mubiao_mm;
    float    qiu_qingjiao_max;
    float    qiu_siqu_mm;
    float    qiu_sulv_max;
    int8_t   qiu_fuhao;
    uint8_t  reserved2[3];
    float    qiu_vel_kp;
    float    qiu_vel_kd;

    uint8_t  qiu_ff_enable;
    uint8_t  qiu_ff_physics_enable;
    int8_t   qiu_ff_sign;
    uint8_t  reserved3;
    float    qiu_ff_k;
    float    qiu_ff_max;

    uint32_t crc;            /* 对以上全部字节（不含本字段） */
} FlashParams_t;

/* 全局运行时标定（Load 可改；Save 写入） */
extern volatile int32_t  jiaodu_pingheng_raw;
extern volatile float    jiaodu_gongzuo_du;

uint32_t Flash_CRC32(const uint8_t *data, uint32_t len);

/* 从 Flash 灌入 sudu_pid / jiaodu_pid / 零点 / 工作区 / 转向，
 * 以及球位置环参数（直接写 qiu_weizhi.h 里的那组全局，不占形参）
 * 成功 1；无数据或 CRC 坏 0（保持调用前的默认）
 * v6 的20ms位置积分会等效换算到10ms；更旧版本按各自迁移规则处理 */
uint8_t FlashParams_Load(PID_t *spd, PID_t *ang, volatile int8_t *dir_out);

/* 把当前 RAM 全部写入 Flash（含位置环）；成功 1 */
uint8_t FlashParams_Save(const PID_t *spd, const PID_t *ang, int8_t dir);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_PARAMS_H */
