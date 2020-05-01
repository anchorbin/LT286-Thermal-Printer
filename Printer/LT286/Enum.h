/*
 * Enum.h
 *
 *  Created on: 7 cze 2017
 *      Author: i5
 */

#ifndef ENUM_H_
#define ENUM_H_

typedef enum STATES
{
	LOW = 0,
	HIGH = 1
} STATE;

typedef enum BITO
{
	LSB = 0,
	MSB = 1
} BITORDER;

typedef enum DIR
{
	BACKWARD = 0,
	FORWARD = 1
} DIRECTION;

typedef enum HEAD_STATE
{
	eHeadDown = 0,
	eHeadUp = 1
} HeadState;

typedef enum PAPER_STATE
{
	ePaperOut = 0,
	ePaperIn = 1
} PaperState;

typedef enum CONTROL_HEAD
{
	eReady = 0,
	eWait = 1,
	eDataStart = 2,
	eHoldLatch = 3,
	eStrobeTrigger = 4,
	eStrobeDisable = 5
} ControlHead;

typedef enum CONTROL_MOTOR
{
	eMotorStart = 0,
	eMotorBackslashForward = 1,
	eMotorBackslashBackward = 2,
	eMotorHold = 3,
	eMotorOff = 4
} ControlMotor;

#endif /* ENUM_H_ */
