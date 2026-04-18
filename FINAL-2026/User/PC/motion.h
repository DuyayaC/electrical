#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include "stdbool.h"

#pragma pack(1)
typedef struct {
    uint8_t  head;      
    float    yaw;       
    float    pitch;
	bool flag;	
	uint8_t  tail;  
} AnglePacket_t;
#pragma pack()

typedef struct
{
	float deltaYaw;
	float deltaPitch;
	bool auto_fire_flag;
}__attribute__((aligned(4))) usb_t;

typedef struct
{
  uint8_t vx_detect;
  uint8_t vy_detect;
  uint8_t omega_detect;
  uint8_t auto_fire_detect;
}__attribute__((aligned(4))) motion_t;

extern motion_t motion;
extern usb_t usb;
extern AnglePacket_t* pAngleData;

uint8_t usbreceive(uint8_t* data, uint16_t len);

#endif