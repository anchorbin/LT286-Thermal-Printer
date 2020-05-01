/*
 * LT286.c
 *
 *  Created on: 7 cze 2017
 *      Author: i5
 */

#include "LT286.h"
#include "Bitmap.h"

const char id_version[] = "Thermal Printer v0.1";

const double ADC_Volt = 3.32;
const double ADC_Max = 4095.0;
const uint32_t ADC_Res[5] = { 32900, 51250, 10000, 51000, 100000 };
const double Beta = 3950.0;
const double Resistance_1 = 30000.0;
const double Temperature_at_R1 = 298.15;

__IO const uint8_t p_Sequence[4][4]=
{
	{ 1, 0, 0, 1 }, //First step
	{ 1, 1, 0, 0 }, //Second step
	{ 0, 1, 1, 0 }, //Third step
	{ 0, 0, 1, 1 }  //Fourth step
};

__IO uint64_t Ticks = 0;

PrinterConfig LT286;

uint8_t p_Init(void)
{
	LT286.BufferLength = 48;
	LT286.StrobesNum = 6;
	LT286.StrobeIndex = 0;
	LT286.BitOrder = MSB;

	LT286.LatchPin = (IO){ MCU_LATCH_GPIO_Port, MCU_LATCH_Pin };
	LT286.DataPin = (IO){ MCU_DATA_GPIO_Port, MCU_DATA_Pin };
	LT286.ClockPin = (IO){ MCU_CLOCK_GPIO_Port, MCU_CLOCK_Pin };

	LT286.MotorA = (IO){ IO_COIL_A1_GPIO_Port, IO_COIL_A1_Pin };
	LT286.MotorAn = (IO){ IO_COIL_A2_GPIO_Port, IO_COIL_A2_Pin };
	LT286.MotorB = (IO){ IO_COIL_B1_GPIO_Port, IO_COIL_B1_Pin };
	LT286.MotorBn = (IO){ IO_COIL_B2_GPIO_Port, IO_COIL_B2_Pin };

	LT286.StrobePin[0] = (IO){ MCU_STB1_GPIO_Port, MCU_STB1_Pin };
	LT286.StrobePin[1] = (IO){ MCU_STB2_GPIO_Port, MCU_STB2_Pin };
	LT286.StrobePin[2] = (IO){ MCU_STB3_GPIO_Port, MCU_STB3_Pin };
	LT286.StrobePin[3] = (IO){ MCU_STB4_GPIO_Port, MCU_STB4_Pin };
	LT286.StrobePin[4] = (IO){ MCU_STB5_GPIO_Port, MCU_STB5_Pin };
	LT286.StrobePin[5] = (IO){ MCU_STB6_GPIO_Port, MCU_STB6_Pin };

	LT286.HeadUp = (IO){ IO_HEAD_GPIO_Port, IO_HEAD_Pin };
	LT286.PaperSensor = (IO){ IO_PAPER_GPIO_Port, IO_PAPER_Pin };

	LT286.Step = 0;
	LT286.StepsMade = 0;
	LT286.BackslashStep = 0;
	LT286.PulseWidth = 3000;
	LT286.DotsActivated = 0;
	LT286.TimePerStep = LT286.PulseWidth;

	LT286.FSM_HEAD = eReady;
	LT286.FSM_MOTOR = eMotorOff;
	LT286.s_Paper = HAL_GPIO_ReadPin(IO_PAPER_GPIO_Port, IO_PAPER_Pin);
	LT286.s_Head = HAL_GPIO_ReadPin(IO_HEAD_GPIO_Port, IO_HEAD_Pin);

	LT286.p_Ready = 1;

	LT286.BitmapPosition = 0;
	LT286.BitmapDone = 1;

	HAL_GPIO_Write(LT286.LatchPin, HIGH);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) LT286.ADC, 3);

	USB_SendString(id_version);
	//p_BackslashRemove();

	return 1;
}

uint8_t p_SingleStep(uint8_t Direction)
{
	if ( p_isHeadReady() )
	{
		if ( Direction == BACKWARD )
		{
			LT286.Step++;

			if (LT286.Step >= 4) LT286.Step = 0;

			HAL_GPIO_Write(LT286.MotorA, p_Sequence[LT286.Step][0]);
			HAL_GPIO_Write(LT286.MotorB, p_Sequence[LT286.Step][1]);
			HAL_GPIO_Write(LT286.MotorAn, p_Sequence[LT286.Step][2]);
			HAL_GPIO_Write(LT286.MotorBn, p_Sequence[LT286.Step][3]);
		}
		else
		{
			LT286.Step--;

			if (LT286.Step < 0) LT286.Step = 3;

			HAL_GPIO_Write(LT286.MotorA, p_Sequence[LT286.Step][0]);
			HAL_GPIO_Write(LT286.MotorB, p_Sequence[LT286.Step][1]);
			HAL_GPIO_Write(LT286.MotorAn, p_Sequence[LT286.Step][2]);
			HAL_GPIO_Write(LT286.MotorBn, p_Sequence[LT286.Step][3]);
		}
		return 1;
	}
	else return 0;
}

void p_BackslashRemove(void)
{
	p_MovePaper(20, BACKWARD);
	p_MovePaper(20, FORWARD);
}

void p_MovePaper(uint32_t Lines, uint8_t Direction)
{
	if( Lines > 0 )
	{
		for ( int i = 0; i < Lines; i++)
		{
			p_SingleStep(Direction);
		}
		p_MotorDisable();
	}
}

void p_MotorDisable(void)
{
	LT286.Step = 0;

	HAL_GPIO_Write(LT286.MotorA, LOW);
	HAL_GPIO_Write(LT286.MotorAn, LOW);
	HAL_GPIO_Write(LT286.MotorB, LOW);
	HAL_GPIO_Write(LT286.MotorBn, LOW);
}

void p_StrobesOff(void)
{
	for ( int i = 0; i < 5; i++)
	{
		HAL_GPIO_Write(LT286.StrobePin[i], LOW);
	}
}

void p_WriteBitToBuffer(uint16_t WriteAt, uint8_t State)
{
	if ( WriteAt <= (LT286.BufferLength * 8) )
	{
		uint8_t Index = WriteAt / 8;
		uint8_t ShiftTo = ( WriteAt - ( Index * 8 ) );

		if ( State == 1 )
		{
			LT286.PrinterBuffer[Index] |= 1 << ShiftTo;
		}
		else if ( State == 0 )
		{
			LT286.PrinterBuffer[Index] &= ~(1 << ShiftTo);
		}
	}
}

void p_WriteByteToBuffer(uint16_t WriteAt, uint8_t Byte)
{
	if ( WriteAt <= LT286.BufferLength )
	{
		LT286.PrinterBuffer[WriteAt] = Byte;
	}
}

void p_ClearBuffer(void)
{
	for ( uint16_t i = 0; i < LT286.BufferLength; i++ )
	{
		LT286.PrinterBuffer[i] = 0x0;
	}
}

uint8_t p_PushData(uint8_t ClearBuffer)
{
	if ( LT286.FSM_HEAD == eReady && p_isHeadReady() )
	{
		LT286.FSM_HEAD = eWait;
		LT286.FSM_MOTOR = eMotorBackslashBackward;

		return 1;
	}
	else return 0;
}

void p_SendData(uint8_t Data)
{
	switch ( LT286.BitOrder )
	{
		case LSB:
		{
			for (uint8_t i = 0; i < 8; i++)
			{
				HAL_GPIO_Write(LT286.DataPin, !!( Data & ( 1 << i )));

				HAL_GPIO_Write(LT286.ClockPin, HIGH);
				HAL_GPIO_Write(LT286.ClockPin, LOW);
			}
		}
		break;
		case MSB:
		{
			for (uint8_t i = 0; i < 8; i++)
			{
				HAL_GPIO_Write(LT286.DataPin, !!( Data & ( 1 << ( 7 - i ))));

				HAL_GPIO_Write(LT286.ClockPin, HIGH);
				HAL_GPIO_Write(LT286.ClockPin, LOW);
			}
		}
		break;
	}
}

uint8_t done = 0;
uint16_t c = 0;

void p_PrintDone(void)
{
	if ( c <= 48*380 )
	{
	p_ClearBuffer();
	for ( int  i = 0; i < 48; i++)
	{
		p_WriteByteToBuffer(i, img[c]);
		 c++;
	}

	LT286.PulseWidth = p_getPulseWidth();
	LT286.TimePerStep = LT286.PulseWidth;

	LT286.FSM_HEAD = eDataStart;
	LT286.FSM_MOTOR = eMotorStart;
	} else { LT286.FSM_MOTOR = eMotorHold; p_StrobesOff(); }
}

void p_PrintBitmap(void)
{
	p_PushData(1);
}

void HAL_GPIO_Write(IO Pin, GPIO_PinState State)
{
	HAL_GPIO_WritePin(Pin.Port, (int)Pin.Pin, State);
}

uint8_t Trigger(__IO uint64_t * Tick, uint64_t Time)
{
	if ( Ticks - *Tick >= Time)
	{
		*Tick = Ticks;

		return 1;
	}
	else return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Every 10 uS
{
	if(htim->Instance == TIM7)
	{
		Ticks++;
		if ( LT286.p_Ready )
		{
			switch ( LT286.FSM_HEAD )
			{
				case eReady:
				{
					//Do nothing
				} break;
				case eWait:
				{
					//Do nothing
				} break;
				case eDataStart:
				{
					LT286.FSM_HEAD = eWait;

					for ( uint8_t Index = 0; Index < LT286.BufferLength; Index++)
					{
						p_SendData(LT286.PrinterBuffer[Index]); //Push data to the printer.
					}

					LT286.PulseWidth = p_getPulseWidth();
					LT286.TimePerStep = LT286.PulseWidth;

					HAL_GPIO_Write(LT286.LatchPin, LOW);

					LT286.FSM_HEAD = eHoldLatch;
				} break;
				case eHoldLatch:
				{
					if ( Trigger(&LT286.Ticks[2], UsToTicks(10)))
					{
						HAL_GPIO_Write(LT286.LatchPin, HIGH);

						LT286.FSM_HEAD = eStrobeTrigger;
					}
				} break;
				case eStrobeTrigger:
				{
					LT286.FSM_HEAD = eWait;
					if ( p_isHeadReady() )
					{
						if ( (LT286.StrobesNum - 1) >= LT286.StrobeIndex)
						{
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex], HIGH);
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex+1], HIGH);
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex+2], HIGH);

							LT286.FSM_HEAD = eStrobeDisable;
						}
					}
				} break;
				case eStrobeDisable:
				{
					if ( Trigger(&LT286.Ticks[3], UsToTicks(LT286.PulseWidth)))
					{
						LT286.FSM_HEAD = eWait;

						{
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex], LOW);
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex+1], LOW);
							HAL_GPIO_Write(LT286.StrobePin[LT286.StrobeIndex+2], LOW);

							if ( LT286.StrobeIndex == 3)
							{
								LT286.StrobeIndex = 0;

								LT286.FSM_HEAD = eReady;

								p_PrintDone();
							}
							else
							{
								LT286.StrobeIndex = LT286.StrobeIndex + 3;

								LT286.FSM_HEAD = eStrobeTrigger;
							}
						}
					}
				} break;
			}
			switch ( LT286.FSM_MOTOR )
			{
				case eMotorOff:
				{
					//Do nothing.
				} break;
				case eMotorBackslashBackward:
				{
					if ( Trigger(&LT286.Ticks[1], UsToTicks(LT286.TimePerStep)))
					{
						uint8_t eState = p_SingleStep(BACKWARD); if (eState == 0) LT286.FSM_MOTOR = eMotorHold;

						if ( LT286.BackslashStep != 20 )
						{
							LT286.BackslashStep++;
						}
						else
						{
							LT286.FSM_MOTOR = eMotorBackslashForward;
						}

					}
				} break;
				case eMotorBackslashForward:
				{
					if ( Trigger(&LT286.Ticks[1], UsToTicks(LT286.TimePerStep)))
					{
						uint8_t eState = p_SingleStep(FORWARD); if (eState == 0) LT286.FSM_MOTOR = eMotorHold;

						if ( LT286.BackslashStep != 40 )
						{
							LT286.BackslashStep++;
						}
						else
						{
							LT286.BackslashStep = 0;

							LT286.FSM_HEAD = eDataStart;
							LT286.FSM_MOTOR = eMotorStart;
						}
					}
				} break;
				case eMotorStart:
				{
					if ( Trigger(&LT286.Ticks[1], UsToTicks(LT286.TimePerStep)))
					{
						uint8_t eState = p_SingleStep(FORWARD);

						if (eState == 0)
						{
							LT286.FSM_MOTOR = eMotorHold;
						}
					}
				} break;
				case eMotorHold:
				{
					if ( Trigger(&LT286.Ticks[1], MsToTicks(50)))
					{
						p_MotorDisable();

						LT286.FSM_MOTOR = eMotorOff;
					}
				} break;
			}
		}
	}
}

uint64_t SecToTicks(uint32_t Sec)
{
	return Sec * 1000000;
}

uint64_t MsToTicks(uint32_t Ms)
{
	return Ms * 100;
}

uint64_t UsToTicks(uint32_t Us)
{
	return Us/10;
}

uint8_t p_GetByte(uint16_t Pos)
{
	if ( Pos <= LT286.BufferLength )
	{
		return LT286.PrinterBuffer[Pos];
	} else return 0;
}

uint8_t p_GetBit(uint16_t Pos)
{
	if ( Pos <= ( LT286.BufferLength * 8))
	{
		uint8_t Index = Pos / 8;
		uint8_t ShiftTo = ( Pos - ( Index * 8 ) );

		return ((LT286.PrinterBuffer[Index] >> ShiftTo) & 1);
	}
	else return 0;
}

uint8_t p_GetBitFromByte(uint8_t B, uint8_t Pos)
{
	if ( Pos >= 0 || Pos <= 7 )
	{
		return (B >> Pos) & 1;
	}
	else return 0;
}

void p_WriteBitToByte(uint8_t Height, uint8_t Value, uint16_t Pos)
{
	if ( Pos <= (LT286.BufferLength * 8) )
	{
		uint8_t Index = Pos / 8;
		uint8_t ShiftTo = ( Pos - ( Index * 8 ) );

		if ( Value == 1 )
		{
			LT286.PrinterBuffer[Index] |= 1 << ShiftTo;
		}
		else if ( Value == 0 )
		{
			LT286.PrinterBuffer[Index] &= ~(1 << ShiftTo);
		}
	}
}

double g_VoltageADC(uint16_t ADC)
{
	return (((double)ADC / ADC_Max) * ADC_Volt);
}

double p_DividerVoltage(uint16_t ADC, uint16_t R1, uint16_t R2)
{
	double r_Vol = g_VoltageADC(ADC);
	double r_Temp = ((r_Vol * (double)(R1+R2)) / (double) R1);

	return r_Temp;
}

uint8_t USB_SendString(const char * Str)
{
	uint8_t s_Res = 0;

	for ( int len = 0; len < LT286.USB_LenTX; len++)
	{
		LT286.USB_TX[len] = 0;
	}


	LT286.USB_LenTX = (uint16_t)sprintf((char *)LT286.USB_TX, "%s\r\n", Str);

	s_Res = CDC_Transmit_FS(LT286.USB_TX, LT286.USB_LenTX);

	return s_Res;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if( GPIO_Pin == IO_PAPER_Pin )
 {
	 LT286.s_Paper = HAL_GPIO_ReadPin(IO_PAPER_GPIO_Port, IO_PAPER_Pin);

	 if ( (uint8_t)HAL_GPIO_ReadPin(IO_PAPER_GPIO_Port, IO_PAPER_Pin) == ePaperIn )
	 {
		 USB_SendString("Wsunieto papier.");
	 }
	 else USB_SendString("Brak papieru.");
 }
 else if ( GPIO_Pin == IO_HEAD_Pin )
 {
	 LT286.s_Head = HAL_GPIO_ReadPin(IO_HEAD_GPIO_Port, IO_HEAD_Pin);

	 if ( (uint8_t)HAL_GPIO_ReadPin(IO_HEAD_GPIO_Port, IO_HEAD_Pin) == eHeadDown )
	 {
		 	USB_SendString("Glowica zamknieta.");
	 }
	 else USB_SendString("Glowica otwarta.");
 }

}

uint8_t USB_ReceivedData(int Len)
{
	if ( strcmp ((const char *)LT286.USB_RX, "Drukuj") == 0)
	{
		c = 0;

		if ( p_PushData(1) )
		{
			USB_SendString("Drukuje");
		} else USB_SendString("Nie moge wydrukowac");
	}

	else if ( strcmp ((const char *)LT286.USB_RX, "Napiecie") == 0)
	{
		char Buferek[64];
		sprintf(Buferek, "Napiecie: %f", p_DividerVoltage(LT286.ADC[1], ADC_Res[0], ADC_Res[1]));

		USB_SendString(Buferek);
	}

	else if ( strcmp ((const char *)LT286.USB_RX, "Temperatura") == 0)
	{
		char Buferek[64];
		sprintf(Buferek, "Rezystor: %f", p_getHeadTemperature());

		USB_SendString(Buferek);
	}

	else if ( strcmp ((const char *)LT286.USB_RX, "Kropki") == 0)
	{
		char Buferek[64];
		sprintf(Buferek, "Kropki: %i", LT286.DotsActivated);

		USB_SendString(Buferek);
	}

	for (uint16_t i = 0; i < Len; i++)
	{
		LT286.USB_RX[i] = 0; // Flush the buffer.
	}

	return 1;
}

uint8_t * USB_RXBuffer(void)
{
	return LT286.USB_RX;
}

uint8_t p_isHeadReady(void)
{
	if ( (LT286.s_Paper == ePaperIn ) && ( LT286.s_Head == eHeadDown ) )
	{
		return 1;
	} else return 0;
}

double p_getHeadTemperature(void)
{
	uint32_t T_Res = (ADC_Res[4] * (( ADC_Max / (double)LT286.ADC[0]) - 1.0));

	double _tVal = (Resistance_1/T_Res);
	double Kelvins = ((Temperature_at_R1 * Beta)/_tVal)/( Beta/_tVal - Temperature_at_R1 );

	return ( Kelvins - 273.15 );
}

int p_getPulseWidth(void)
{
	double VH = p_DividerVoltage(LT286.ADC[1], ADC_Res[0], ADC_Res[1]);
    double V = -0.0225 * (VH*VH) + 1.7745 * VH - 3.41;
    double retVal = 0.37 * (((192.0 * 0.48) + 142.0 + 15.0 + 10.0) * ((192.0 * 0.48) + 142.0 + 15.0 + 10.0))/((V*V) * 142.0 );

    return retVal * 1000;
}

uint8_t p_bitsCount(uint8_t byte)
{
	byte = byte - ((byte >> 1) & 0x55555555);
	byte = (byte & 0x33333333) + ((byte >> 2) & 0x33333333);
    return (((byte + (byte >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

