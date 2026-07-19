/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң࠘ǷԦmìң࠘Ƿˇͨڽ`̆SBUSքЭөԫˤì{ԃDMAԫˤ׽ʽޚԼCPU
  *             ؊Դì{ԃԮࠚࠕАא׏4-ǰԦmگ˽ìͬʱ͡٩һЩִПטǴDMAìԮࠚ
  *             ք׽ʽѣ֤Ɉӥюքψ֨єc
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Ϊԉ
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
	*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH_SBUS 25u
#define RC_FRAME_LENGTH_DBUS 18u
/* 兼容旧代码：默认仍按 25 字节 SBUS 命名 */
#define RC_FRAME_LENGTH RC_FRAME_LENGTH_SBUS

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/*-----------*/
#define Rocker_Mid 1024
#define Rocker_Delt_Range 783

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
          int16_t ch[10];
          char s[2];
        } rc;
        __packed struct
        {
          uint16_t v;
        } key;

} RC_ctrl_t;

/* (debug counters removed) */


/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң࠘ǷԵʼۯ
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ܱȡң࠘Ƿ˽ߝָ֫
  * @param[in]      none
  * @retval         ң࠘Ƿ˽ߝָ֫
  */
extern const RC_ctrl_t *get_remote_control_point(void);

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң࠘ǷЭөޢ϶
  * @param[in]      sbus_buf: ԭʺ˽ߝָ֫
  * @param[out]     rc_ctrl: ң࠘Ƿ˽ߝָ
  * @retval         none
  */
//extern void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);




#endif

