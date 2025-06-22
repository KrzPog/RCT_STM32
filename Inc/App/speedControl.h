#ifndef _SPEED_CONTROL_H
#define _SPEED_CONTROL_H

#include "bldc_interface.h"
#include "bldc_interface_uart.h"

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "App/pid.h"
#include "ModbusRegisters/reg_holding.h"

#define SPEED_UPDATE_PERIOD_MS 20

extern bool elevLimitMinReached;
extern bool elevLimitMaxReached;

extern bool Rot_UART_VESC_Enabled;

extern int16_t speedCV_rot;
extern int16_t speedCV_elev;

void initRotSpeedControl(void);
void initElevSpeedControl(void);

int16_t getRotSpeedCV(void);
int16_t getElevSpeedCV(void);

void setRotSpeedPWM(int16_t speedCV);
void setElevSpeedPWM(int16_t speedCV);

void setRotSpeedUART(int16_t speedCV);
void setElevSpeedUART(int16_t speedCV);

static void rot_vesc_send_packet_rot(unsigned char *data, unsigned int len);
void rot_vesc_uart_rx_callback(UART_HandleTypeDef *huart);

#endif