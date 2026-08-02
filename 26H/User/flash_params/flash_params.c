#include "flash_params.h"
#include "qiu_weizhi.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

_Static_assert(sizeof(FlashParamsV7_t) == 88U, "FlashParamsV7_t layout changed");
_Static_assert(sizeof(FlashParams_t) == 100U, "FlashParams_t layout changed");

/* 运行时标定：上电先等于宏默认，Load 成功后可被 Flash 覆盖 */
volatile int32_t jiaodu_pingheng_raw = JIAODU_PINGHENG_RAW;
volatile float   jiaodu_gongzuo_du   = JIAODU_GONGZUO_DU;

/**
 * @brief 与 ytbeif 相同的软件 CRC32
 */
uint32_t Flash_CRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    int j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/**
 * @brief v2 旧存档迁移：只灌速度/角度/标定，位置环保持宏默认
 *        今晚调好的 P55/I0.55 + J0.12/L0.10 靠这条路活下来
 */
static uint8_t flash_load_v2(PID_t *spd, PID_t *ang, volatile int8_t *dir_out)
{
    const FlashParamsV2_t *p = (const FlashParamsV2_t *)FLASH_SAVE_ADDR;
    uint32_t crc = Flash_CRC32((const uint8_t *)p,
                               (uint32_t)(sizeof(FlashParamsV2_t) - sizeof(uint32_t)));

    if (crc != p->crc)
    {
        printf("# FLASH v2 CRC fail, use defaults\r\n");
        return 0U;
    }

    if (spd != 0)
    {
        spd->Kp = p->spd_kp;
        spd->Ki = p->spd_ki;
        spd->Kd = p->spd_kd;
        PID_Reset(spd);
    }
    if (ang != 0)
    {
        ang->Kp = p->ang_kp;
        ang->Ki = p->ang_ki;
        ang->Kd = p->ang_kd;
        PID_Reset(ang);
    }

    jiaodu_pingheng_raw = p->pingheng_raw;
    if (p->gongzuo_du > 0.1f && p->gongzuo_du < 80.0f)
    {
        jiaodu_gongzuo_du = p->gongzuo_du;
    }
    if (dir_out != 0)
    {
        *dir_out = (p->zhuanxiang < 0) ? (int8_t)-1 : (int8_t)1;
    }

    printf("# FLASH load OK (v2 migrated, qiu=defaults)\r\n");
    printf("#   spd P=%.2f I=%.2f D=%.2f | ang J=%.3f K=%.3f L=%.3f\r\n",
           p->spd_kp, p->spd_ki, p->spd_kd,
           p->ang_kp, p->ang_ki, p->ang_kd);
    printf("#   zero raw=%ld work=±%.1fdeg dir=%d | SAVE once to upgrade v7\r\n",
           (long)p->pingheng_raw, (double)p->gongzuo_du,
           (int)((p->zhuanxiang < 0) ? -1 : 1));
    return 1U;
}

/* v3/v4 的位置环参数分别按20ms/100ms离散周期调过，不能直接用于10ms。
 * 保留已调好的速度/角度/标定，位置环保持 QiuWeizhi_Init() 设置的10ms默认值。 */
static uint8_t flash_load_legacy_qiu(PID_t *spd, PID_t *ang,
                                     volatile int8_t *dir_out, uint32_t old_version)
{
    const FlashParams_t *p = (const FlashParams_t *)FLASH_SAVE_ADDR;
    uint32_t crc = Flash_CRC32((const uint8_t *)p,
                               (uint32_t)(sizeof(FlashParams_t) - sizeof(uint32_t)));

    if (crc != p->crc)
    {
        printf("# FLASH v%lu CRC fail, use defaults\r\n", (unsigned long)old_version);
        return 0U;
    }
    if (spd != 0)
    {
        spd->Kp = p->spd_kp;
        spd->Ki = p->spd_ki;
        spd->Kd = p->spd_kd;
        PID_Reset(spd);
    }
    if (ang != 0)
    {
        ang->Kp = p->ang_kp;
        ang->Ki = p->ang_ki;
        ang->Kd = p->ang_kd;
        PID_Reset(ang);
    }
    jiaodu_pingheng_raw = p->pingheng_raw;
    if (p->gongzuo_du > 0.1f && p->gongzuo_du < 80.0f)
    {
        jiaodu_gongzuo_du = p->gongzuo_du;
    }
    if (dir_out != 0)
    {
        *dir_out = (p->zhuanxiang < 0) ? (int8_t)-1 : (int8_t)1;
    }

    printf("# FLASH load OK (v%lu migrated, qiu=defaults) | SAVE→v7\r\n",
           (unsigned long)old_version);
    printf("#   zero raw=%ld work=±%.1fdeg dir=%d\r\n",
           (long)p->pingheng_raw, (double)p->gongzuo_du,
           (int)((p->zhuanxiang < 0) ? -1 : 1));
    return 1U;
}

/* v5：位置/限幅已是串级量级，但没有 C/O；CRC 按 V5 结构校验 */
static uint8_t flash_load_v5(PID_t *spd, PID_t *ang, volatile int8_t *dir_out)
{
    const FlashParamsV5_t *p = (const FlashParamsV5_t *)FLASH_SAVE_ADDR;
    uint32_t crc = Flash_CRC32((const uint8_t *)p,
                               (uint32_t)(sizeof(FlashParamsV5_t) - sizeof(uint32_t)));

    if (crc != p->crc)
    {
        printf("# FLASH v5 CRC fail, use defaults\r\n");
        return 0U;
    }
    if (spd != 0)
    {
        spd->Kp = p->spd_kp;
        spd->Ki = p->spd_ki;
        spd->Kd = p->spd_kd;
        PID_Reset(spd);
    }
    if (ang != 0)
    {
        ang->Kp = p->ang_kp;
        ang->Ki = p->ang_ki;
        ang->Kd = p->ang_kd;
        PID_Reset(ang);
    }
    jiaodu_pingheng_raw = p->pingheng_raw;
    if (p->gongzuo_du > 0.1f && p->gongzuo_du < 80.0f)
    {
        jiaodu_gongzuo_du = p->gongzuo_du;
    }
    if (dir_out != 0)
    {
        *dir_out = (p->zhuanxiang < 0) ? (int8_t)-1 : (int8_t)1;
    }
    if (p->qiu_kp >= 0.5f && p->qiu_kp < 50.0f)
    {
        qiu_pid.Kp = p->qiu_kp;
        qiu_pid.Ki = p->qiu_ki * 0.5f;
        qiu_pid.Kd = p->qiu_kd;
        PID_Reset(&qiu_pid);
    }
    qiu_mubiao_mm = p->qiu_mubiao_mm;
    if (p->qiu_qingjiao_max > 0.1f && p->qiu_qingjiao_max <= 80.0f)
    {
        qiu_qingjiao_max = p->qiu_qingjiao_max;
    }
    if (p->qiu_siqu_mm >= 0.0f && p->qiu_siqu_mm < 200.0f)
    {
        qiu_siqu_mm = p->qiu_siqu_mm;
    }
    if (p->qiu_sulv_max > 0.1f && p->qiu_sulv_max <= 2000.0f)
    {
        qiu_sulv_max = p->qiu_sulv_max;
    }
    qiu_fuhao = (p->qiu_fuhao < 0) ? (int8_t)-1 : (int8_t)1;
    /* C/O 保持 QiuWeizhi_Init 默认 */

    printf("# FLASH load OK (v5→10ms, vel defaults) | SAVE once to upgrade v7\r\n");
    printf("#   pos X=%.2f Z=%.2f H=%.1f U=%.1f M=%.0f\r\n",
           (double)qiu_pid.Kp, (double)qiu_pid.Kd,
           (double)qiu_qingjiao_max, (double)qiu_siqu_mm, (double)qiu_sulv_max);
    return 1U;
}

/**
 * @brief 开机 / R 命令：从 Sector7 加载参数
 *        v8 全量；v6/v7 自动迁移；更旧版本按原规则加载
 */
uint8_t FlashParams_Load(PID_t *spd, PID_t *ang, volatile int8_t *dir_out)
{
    const FlashParams_t *p = (const FlashParams_t *)FLASH_SAVE_ADDR;
    const FlashParamsV7_t *p7 = (const FlashParamsV7_t *)FLASH_SAVE_ADDR;
    uint32_t crc;
    uint8_t migrate_v6 = 0U;
    uint8_t migrate_v7 = 0U;

    if (p->magic != FLASH_SAVE_MAGIC)
    {
        printf("# FLASH empty, use defaults (P%.0f I%.2f D%.0f)\r\n",
               (double)SUDU_KP_DEFAULT, (double)SUDU_KI_DEFAULT,
               (double)SUDU_KD_DEFAULT);
        return 0U;
    }

    if (p->version == 2U)
    {
        return flash_load_v2(spd, ang, dir_out);
    }
    if (p->version == 3U)
    {
        return flash_load_legacy_qiu(spd, ang, dir_out, 3U);
    }
    if (p->version == 4U)
    {
        return flash_load_legacy_qiu(spd, ang, dir_out, 4U);
    }
    if (p->version == 5U)
    {
        return flash_load_v5(spd, ang, dir_out);
    }
    if (p->version == 6U)
    {
        migrate_v6 = 1U;
    }
    else if (p->version == 7U)
    {
        migrate_v7 = 1U;
    }
    else if (p->version != FLASH_SAVE_VERSION)
    {
        printf("# FLASH ver=%lu want=%u, use defaults\r\n",
               (unsigned long)p->version, (unsigned)FLASH_SAVE_VERSION);
        return 0U;
    }

    if (migrate_v6 || migrate_v7)
    {
        crc = Flash_CRC32((const uint8_t *)p7,
                          (uint32_t)(sizeof(FlashParamsV7_t) - sizeof(uint32_t)));
    }
    else
    {
        crc = Flash_CRC32((const uint8_t *)p,
                          (uint32_t)(sizeof(FlashParams_t) - sizeof(uint32_t)));
    }
    if (crc != ((migrate_v6 || migrate_v7) ? p7->crc : p->crc))
    {
        printf("# FLASH CRC fail, use defaults\r\n");
        return 0U;
    }

    if (spd != 0)
    {
        spd->Kp = p->spd_kp;
        spd->Ki = p->spd_ki;
        spd->Kd = p->spd_kd;
        PID_Reset(spd);
    }
    if (ang != 0)
    {
        ang->Kp = p->ang_kp;
        ang->Ki = p->ang_ki;
        ang->Kd = p->ang_kd;
        PID_Reset(ang);
    }

    jiaodu_pingheng_raw = p->pingheng_raw;
    if (p->gongzuo_du > 0.1f && p->gongzuo_du < 80.0f)
    {
        jiaodu_gongzuo_du = p->gongzuo_du;
    }
    if (dir_out != 0)
    {
        *dir_out = (p->zhuanxiang < 0) ? (int8_t)-1 : (int8_t)1;
    }

    /* 位置环已改为「mm → 目标球速 mm/s」，Kp 量级约 1~10。
     * 旧固件存的是「mm → 倾角°」的 0.0x，直接灌会几乎不动。
     * Kp < 0.5 视为旧档/坏档，保留 QiuWeizhi_Init 默认。 */
    if (p->qiu_kp >= 0.5f && p->qiu_kp < 50.0f)
    {
        qiu_pid.Kp = p->qiu_kp;
        qiu_pid.Ki = migrate_v6 ? (p->qiu_ki * 0.5f) : p->qiu_ki;
        qiu_pid.Kd = p->qiu_kd;
        PID_Reset(&qiu_pid);
    }
    else
    {
        printf("# FLASH qiu Kp=%.3f looks pre-cascade, keep defaults\r\n",
               (double)p->qiu_kp);
    }

    qiu_mubiao_mm = p->qiu_mubiao_mm;
    if (p->qiu_qingjiao_max > 0.1f && p->qiu_qingjiao_max <= 80.0f)
    {
        qiu_qingjiao_max = p->qiu_qingjiao_max;
    }
    if (p->qiu_siqu_mm >= 0.0f && p->qiu_siqu_mm < 200.0f)
    {
        qiu_siqu_mm = p->qiu_siqu_mm;
    }
    if (p->qiu_sulv_max > 0.1f && p->qiu_sulv_max <= 2000.0f)
    {
        qiu_sulv_max = p->qiu_sulv_max;
    }
    qiu_fuhao = (p->qiu_fuhao < 0) ? (int8_t)-1 : (int8_t)1;

    if (p->qiu_vel_kp > 1e-6f && p->qiu_vel_kp < 1.0f)
    {
        qiu_vel_kp = p->qiu_vel_kp;
    }
    if (p->qiu_vel_kd >= 0.0f && p->qiu_vel_kd < 1.0f)
    {
        qiu_vel_kd = p->qiu_vel_kd;
    }

    if (!migrate_v6 && !migrate_v7)
    {
        qiu_ff_enable = p->qiu_ff_enable ? 1U : 0U;
        qiu_ff_physics_enable = p->qiu_ff_physics_enable ? 1U : 0U;
        qiu_ff_sign = (p->qiu_ff_sign < 0) ? (int8_t)-1 : (int8_t)1;
        if (p->qiu_ff_k >= 0.0f && p->qiu_ff_k <= 8.0f)
        {
            qiu_ff_k = p->qiu_ff_k;
        }
        if (p->qiu_ff_max >= 0.0f && p->qiu_ff_max <= jiaodu_gongzuo_du)
        {
            qiu_ff_max = p->qiu_ff_max;
        }
    }

    if (migrate_v6)
    {
        printf("# FLASH load OK (v6->v8, pos Ki %.4f->%.4f)\r\n",
               (double)p->qiu_ki, (double)qiu_pid.Ki);
    }
    else if (migrate_v7)
    {
        printf("# FLASH load OK (v7->v8, IMU FF=tuned defaults)\r\n");
    }
    else
    {
        printf("# FLASH load OK (v8)\r\n");
    }
    printf("#   spd P=%.2f I=%.2f D=%.2f\r\n",
           p->spd_kp, p->spd_ki, p->spd_kd);
    printf("#   ang J=%.3f K=%.3f L=%.3f\r\n",
           p->ang_kp, p->ang_ki, p->ang_kd);
    printf("#   zero raw=%ld work=±%.1fdeg dir=%d\r\n",
           (long)p->pingheng_raw, (double)p->gongzuo_du,
           (int)((p->zhuanxiang < 0) ? -1 : 1));
    printf("#   pos X=%.2f Y=%.3f Z=%.2f | vel C=%.3f O=%.3f\r\n",
           (double)qiu_pid.Kp, (double)qiu_pid.Ki, (double)qiu_pid.Kd,
           (double)qiu_vel_kp, (double)qiu_vel_kd);
    printf("#   G=%.1fmm H=%.1fdeg U=%.1fmm M=%.0f N=%d\r\n",
           (double)qiu_mubiao_mm, (double)qiu_qingjiao_max,
           (double)qiu_siqu_mm, (double)qiu_sulv_max, (int)qiu_fuhao);
    printf("#   ff FE=%u FP=%u FK=%.2f FL=%.1f FS=%d\r\n",
           (unsigned)qiu_ff_enable, (unsigned)qiu_ff_physics_enable,
           (double)qiu_ff_k, (double)qiu_ff_max, (int)qiu_ff_sign);

    if (migrate_v6 || migrate_v7)
    {
        if (!FlashParams_Save(spd, ang, (dir_out != 0) ? *dir_out : 1))
        {
            printf("# FLASH auto-upgrade to v8 failed\r\n");
        }
    }
    return 1U;
}

/**
 * @brief SAVE / W：擦 Sector7 后写入当前全部参数
 *        关中断 + Unlock/Erase/Program/Lock，与 ytbeif 同流程
 */
uint8_t FlashParams_Save(const PID_t *spd, const PID_t *ang, int8_t dir)
{
    FlashParams_t params;
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0U;
    uint32_t *src;
    uint32_t addr;
    uint32_t i;
    uint32_t words;

    memset(&params, 0, sizeof(params));
    params.magic   = FLASH_SAVE_MAGIC;
    params.version = FLASH_SAVE_VERSION;

    params.spd_kp = (spd != 0) ? spd->Kp : SUDU_KP_DEFAULT;
    params.spd_ki = (spd != 0) ? spd->Ki : SUDU_KI_DEFAULT;
    params.spd_kd = (spd != 0) ? spd->Kd : SUDU_KD_DEFAULT;
    params.ang_kp = (ang != 0) ? ang->Kp : JIAODU_KP_DEFAULT;
    params.ang_ki = (ang != 0) ? ang->Ki : JIAODU_KI_DEFAULT;
    params.ang_kd = (ang != 0) ? ang->Kd : JIAODU_KD_DEFAULT;

    params.pingheng_raw = jiaodu_pingheng_raw;
    params.gongzuo_du   = jiaodu_gongzuo_du;
    params.zhuanxiang   = (dir < 0) ? (int8_t)-1 : (int8_t)1;

    /* 位置环/球速环走全局，不额外加形参 */
    params.qiu_kp           = qiu_pid.Kp;
    params.qiu_ki           = qiu_pid.Ki;
    params.qiu_kd           = qiu_pid.Kd;
    params.qiu_mubiao_mm    = qiu_mubiao_mm;
    params.qiu_qingjiao_max = qiu_qingjiao_max;
    params.qiu_siqu_mm      = qiu_siqu_mm;
    params.qiu_sulv_max     = qiu_sulv_max;
    params.qiu_fuhao        = (qiu_fuhao < 0) ? (int8_t)-1 : (int8_t)1;
    params.qiu_vel_kp       = qiu_vel_kp;
    params.qiu_vel_kd       = qiu_vel_kd;
    params.qiu_ff_enable         = qiu_ff_enable ? 1U : 0U;
    params.qiu_ff_physics_enable = qiu_ff_physics_enable ? 1U : 0U;
    params.qiu_ff_sign           = (qiu_ff_sign < 0) ? (int8_t)-1 : (int8_t)1;
    params.qiu_ff_k              = qiu_ff_k;
    params.qiu_ff_max            = qiu_ff_max;
    /* qiu_moshi 故意不存：开机自己动起来太危险，永远手动 B1 */

    params.crc = Flash_CRC32((const uint8_t *)&params,
                             (uint32_t)(sizeof(params) - sizeof(uint32_t)));

    __disable_irq();
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        __enable_irq();
        printf("# FLASH unlock fail\r\n");
        return 0U;
    }

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector       = FLASH_SECTOR_7;
    erase.NbSectors    = 1U;

    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        printf("# FLASH erase fail page_err=%lu\r\n",
               (unsigned long)page_err);
        return 0U;
    }

    src   = (uint32_t *)&params;
    addr  = FLASH_SAVE_ADDR;
    words = (uint32_t)((sizeof(params) + 3U) / 4U);
    for (i = 0U; i < words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            __enable_irq();
            printf("# FLASH program fail @0x%08lX\r\n",
                   (unsigned long)addr);
            return 0U;
        }
        addr += 4U;
    }

    HAL_FLASH_Lock();
    __enable_irq();

    printf("# FLASH SAVE OK (reboot will auto-load)\r\n");
    printf("#   spd P=%.2f I=%.2f D=%.2f\r\n",
           params.spd_kp, params.spd_ki, params.spd_kd);
    printf("#   ang J=%.3f K=%.3f L=%.3f\r\n",
           params.ang_kp, params.ang_ki, params.ang_kd);
    printf("#   zero raw=%ld work=±%.1fdeg dir=%d\r\n",
           (long)params.pingheng_raw, (double)params.gongzuo_du,
           (int)params.zhuanxiang);
    printf("#   qiu X=%.3f Y=%.3f Z=%.3f G=%.1fmm H=%.1fdeg"
           " U=%.1fmm M=%.0f N=%d\r\n",
           (double)params.qiu_kp, (double)params.qiu_ki, (double)params.qiu_kd,
           (double)params.qiu_mubiao_mm, (double)params.qiu_qingjiao_max,
           (double)params.qiu_siqu_mm, (double)params.qiu_sulv_max,
           (int)params.qiu_fuhao);
    printf("#   ff FE=%u FP=%u FK=%.2f FL=%.1f FS=%d\r\n",
           (unsigned)params.qiu_ff_enable,
           (unsigned)params.qiu_ff_physics_enable,
           (double)params.qiu_ff_k, (double)params.qiu_ff_max,
           (int)params.qiu_ff_sign);
    return 1U;
}
