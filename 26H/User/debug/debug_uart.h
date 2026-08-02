#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H

#include <stdint.h>
#include "usart.h"

/* 串口1调试 + 在线调参，115200 8N1，PA9=TX PA10=RX
 *
 * printf 已重定向到 USART1（见 debug_uart.c 的 __io_putchar）。
 * printf 是阻塞的，所以打印一律放主循环，不进中断：
 * 115200下发70字节要约6ms，塞进2ms速度环的中断里会直接顶飞节拍
 *
 * ── 输出格式 ────────────────────────────────────────────────
 *
 * 两种模式互斥，A 命令切换。当前默认是角度标定模式。
 *
 * 【A1 角度标定/联调模式】100ms一条，其他全静音：
 *   # raw=6420 ang=282.1 turn=0 lim=OK dn=618 up=1037 vis=12.34 conn=1 vrx=88
 *   lim   OK=行程内 HI=撞上限 LO=撞下限
 *   dn/up 距软限位下/上端 tick，负数=已在软限位外
 *   vis   球位置 mm（USART2 解析），conn=超时内有数据，vrx=收帧计数
 *
 * 【A0 PID调参模式】数据行 100ms 一条：
 *   tgt:50.0,act:48,volt:8432,err:2.0,isum:1250
 *   诊断 # 行含 lim/vis，便于 E1 V20 联调
 *
 * 标签名和顺序别改，绘图软件靠它对应通道。
 * 其他一切输出都以 # 开头（角度、诊断、命令回显、参数查询），
 * 而且用 = 号不用 : 号，双重区分，不会被当成数据通道
 *
 * ── 调参命令：字母+数值，回车结尾，大小写都认 ────────────────
 *
 *   P50      速度环 Kp        改完立刻生效
 *   I2       速度环 Ki        会自动清积分（见下）
 *   D6       速度环 Kd
 *
 *   V50      目标转速 50 RPM（进入速度模式，角度环让路）
 *   T0/T3    角度环目标：相对水平倾角(度)，T0=水平 raw6493，限±14°
 *   E1 / E0  使能 / 停机
 *   A2 / A1 / A0  IMU标定 / 角度打印 / PID数据行
 *   A2字段：ax/ay/az/iage/ist/ierr/ff/iraw/ipkt/ibad/irxs/itx
 *   J/K/L    角度环 Kp/Ki/Kd（5ms）
 *   P/I/D    速度环 Kp/Ki/Kd（2ms）
 *
 *   B1 / B0  球位置环 开/关（10ms/100Hz，开时自动进角度模式）
 *   X/Y/Z    位置环 Kp/Ki/Kd
 *   G50      目标球位置 mm（0=水管中心）
 *   H6       输出倾角限幅 °（大步进晃先收窄这个）
 *   U0       位置死区 mm，0=关
 *   M60      倾角变化率限幅 °/s
 *   N1/N-1   推球方向标定（球一路跑到底不回头就敲反的）
 *   FE/FP/FK/FL/FS 加速度前馈使能/物理模式/增益/限幅/方向
 *
 *   F1 / F-1 转向标定
 *   W / SAVE 保存当前全部 PID+零点+工作区+转向到 Flash
 *   R / LOAD 从 Flash 重新加载
 *   SP1/SP0  开启/关闭周期数据刷屏（命令回显和 Q 查询不受影响）
 *   S        急停
 *   Q        查询
 *
 * ── 角度限位 ────────────────────────────────────────────────
 *
 * 极限值在 pid.h 的 JIAODU_* 宏里，手动标定实测 raw 5702~7557。
 * 软限位往里退100tick，撞上后只拦"继续往外"的方向，
 * 往里回的永远放行——所以撞了不用敲命令，反向给目标就能退回来
 *
 * ── 其他 ────────────────────────────────────────────────────
 *
 * 为什么改 Ki 要清积分：旧的 err_sum 是按老 Ki 累出来的。
 * 比如 err_sum=5000，Ki 从 2 改到 4，积分项输出瞬间从 10000
 * 跳到 20000，电机会猛窜一下。改 Kp/Kd 不用清，它们不含历史量
 *
 * 默认速度环 P55 I0.55 D0。调完发 SAVE（或 W），掉电后开机自动 Load
 * ─────────────────────────────────────────────────────────── */

#define DBG_RX_BUF_SIZE  24

extern uint8_t          dbg_rx_byte;               // 单字节接收暂存
extern uint8_t          dbg_rx_buf[DBG_RX_BUF_SIZE];
extern volatile uint8_t dbg_rx_idx;
extern volatile uint8_t dbg_cmd_ready;             // 1=收到完整一行
/* TIM7按分频周期置位，主循环打印后清零。
 * 用计数器而不是单标志：主循环没跟上时能看出丢了几帧，
 * 单标志会静默丢弃，表现就是"打印不完整"但查不出原因 */
extern volatile uint8_t dbg_print_pending;
extern volatile uint32_t dbg_print_lost;           // 累计丢帧数
extern volatile uint8_t  dbg_show_angle;           // 0=位置环，1=角度，2=IMU标定
extern volatile uint8_t  dbg_stream_enable;         // 默认0；1=周期刷屏，0=仅命令回显

void Debug_RxStart(void);                  // 启动接收，main里调一次
void Debug_RxByte(void);                   // 中断回调里调，只攒字符
void Debug_ParseCommand(void);             // 主循环轮询，有命令就解析
void Debug_PrintParams(void);              // 打印当前参数（Q命令）
void dianji_dayin(void);                   // 打印一行状态，主循环里调

#endif
