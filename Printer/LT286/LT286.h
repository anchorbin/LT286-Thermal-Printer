/*
 * LT286.h
 *
 *  Created on: 7 cze 2017
 *      Author: i5
 */

#ifndef LT286_H_
#define LT286_H_

#include "stm32f1xx_hal.h"
#include "Enum.h"
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>

typedef struct
{
	GPIO_TypeDef * Port;
	uint16_t * Pin;
} IO;

typedef struct
{
	uint8_t p_Ready; 			// Is printer ready

	uint16_t BufferLength; 		// 48 bytes - 384 bits, 384 dots/line.
	uint8_t StrobesNum; 		// Number of strobes.
	uint8_t PrinterBuffer[48];  // Buffer send to the the thermal head.
	uint8_t BitOrder; 			// Bit order, LSB or MSB.

	IO MotorA; 					// Coil A - stepper motor
	IO MotorAn; 				// Coil An - stepper motor
	IO MotorB; 					// Coil B - stepper motor
	IO MotorBn; 				// Coil Bn - stepper motor

	IO LatchPin; 				// Latch Pin
	IO DataPin; 				// Data Pin
	IO ClockPin; 				// Clock Pin

	IO StrobePin[6]; 			// One strobe is 64 dots, there are 6 strobes in total 64 * 6 = 384 dots..
	uint8_t StrobeIndex; 		// Current strobe activated.

	IO HeadUp; 					// Head up input.
	IO PaperSensor; 			// Paper sensor input.

	uint8_t s_Paper;			// Paper sensor state.
	uint8_t s_Head;				// Head open sensor state.

	int Step; 					// Current step;
	int8_t BackslashStep; 		// Backslash step 20-20
	uint32_t StepsMade; 		// Total steps made by the motor;
	uint8_t BitmapDone;

	uint32_t TimePerStep;		// Time per step in us.
	uint32_t PulseWidth;		// Time of printing in us.
	uint16_t DotsActivated;		// Dots activated 0-384.

	uint8_t USB_TX[512]; 		// Buffer containing data to send.
	int USB_LenTX;   			// Length of transmitted message
	int USB_LenRX;   			// Length of received message.
	uint8_t USB_RX[512];  		// Buffer containing data to receive.

	//uint8_t Buffer_I[256][48]; //First Buffer 256 lines
	//uint8_t Buffer_II[256][48]; //Second Buffer 256 lines

	uint16_t BitmapPosition;

	ControlHead FSM_HEAD; 		// Current state of FSM - Head
	ControlMotor FSM_MOTOR; 	// Current state of FSM - Motor

	uint16_t ADC[3]; 			// DMA - ADC mem. ADC[0] - Head temp., ADC[1] - Batt volt., ADC[2] - MCU temp.

	__IO uint64_t Ticks[6];
} PrinterConfig;

uint8_t p_Init(void); //Init Printer.
uint8_t p_SingleStep(uint8_t Direction); //Make a single step.
void p_MovePaper(uint32_t Lines, uint8_t Direction); //move paper by given lines.
void p_BackslashRemove(void); //Remove backslash to prevent the deterioration of print quality by making 20 steps backward and forward.
uint8_t Trigger(__IO uint64_t * Tick, uint64_t Time);
void p_WriteBitToBuffer(uint16_t WriteAt, uint8_t State); //Write bit to the buffer at position from 0-384.
void p_WriteByteToBuffer(uint16_t WriteAt, uint8_t Byte); //Write byte to the buffer at position from 0-48.
uint8_t p_PushData(uint8_t ClearBuffer); //Send the buffer to the printer head, if ClearBuffer == 1 then clean the buffer.
void p_StrobesOff(void);
void p_SendData(uint8_t Data); //Send data.
void p_PrintDone(void);
void p_PrintBitmap();
void HAL_GPIO_Write(IO Pin, GPIO_PinState State);
uint64_t SecToTicks(uint32_t Sec); //Convert seconds to ticks, 1 tick is every 10us.
uint64_t MsToTicks(uint32_t Ms);  //Convert ms to ticks, 1 tick is every 10us.
uint64_t UsToTicks(uint32_t Us); //Convert us to ticks, 1 tick is every 10us.
uint8_t p_GetByte(uint16_t Pos); //Get Byte at given position, 0-48.
uint8_t p_GetBit(uint16_t Pos); //Get bit at given position, 0-384.
uint8_t p_GetBitFromByte(uint8_t B, uint8_t Pos); //Get bit at given position from given byte;
void p_WriteBitToByte(uint8_t Height, uint8_t Value, uint16_t Pos);
void p_MotorDisable(void); //Turn off the motor to prevent heating.
double p_DividerVoltage(uint16_t ADC, uint16_t R1, uint16_t R2); //Get voltage of voltage divider.
double g_VoltageADC(uint16_t ADC); //Get Voltage from ADC Value.
uint8_t USB_SendString(const char * Str); //Send string - CDC class
uint8_t USB_ReceivedData(int Len); // Received data - CDC class
uint8_t * USB_RXBuffer(void); // Return pointer of RX buffer, so we can copy data from buffer of USB Driver.
uint8_t p_isHeadReady(void); // Is head ready to print - paper inserted & head down.
double p_getHeadTemperature(void); // Get temperature of the thermal head - in celsius.
int p_getPulseWidth(void);
uint8_t p_bitsCount(uint8_t byte);


#endif /* LT286_H_ */
