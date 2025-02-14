
/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *    Acknowledgements to the innovative development, support and guidance by
 *    Markus Dauberschmidt for development of the Pendulum Swing Up algorithm
 *    Please see https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 *              Motor Control Curriculum Feedback Control System
 *
 * Includes:
 * 		Stepper Motor Control interface based on the IHM01A1 and Nucleo F401RE
 * 		Optical Encoder interface supported by the Nucleo F401RE
 * 		Primary PID controller for Pendulum Angle Control
 * 		Secondary PID controller for Rotor Angle Control
 * 		User interfaces for remote access to system configuration including
 * 				Stepper Motor speed profile, current limits, and others
 * 				Control system parameters
 *
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V2.0
 * @date    January 13, 2021
 *
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 * @author  IPC Rennes
 * @version V1.10.0
 * @date    March 16th, 2018
 * @brief   This example shows how to use 1 IHM01A1 expansion board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/*
 * Integrated Rotary Inverted Pendulum System Configuration
 *
 * ********** Primary Components *************************
 * Processor:			Nucleo F401RE
 * Motor Interface: 	IHM01A1 Stepper Motor Controller
 * Motor Power Supply:	12V 2A Supply
 * Encoder:				LPD3806-600BM-G5-24C 600 Pulse Per Revolution Incremental Rotary Encoder
 * Stepper Motor:		NEMA-17 17HS19-2004S  Stepper Motor
 *
 *********** Stepper Lead Assignment *********************
 *
 * Lead Color	IHM01A1 Terminal
 * 	 Blue			-A
 * 	 Red			+A
 * 	 Green			-B
 * 	 Black			+B
 *
 * Stepper Lead Extension Cable (if present) replaces Blue with White
 *
 * Caution: Please note that motors have been receieved from the vendor showing
 * reversal of Motor White and Motor Red.  Check rotor operation after assembly
 * and in initial testing.
 *
 * ********** Encoder Lead Assignment *********************
 *
 * Lead Color	Nucleo F401RE Terminal
 * 	Red 			5V
 * 	Black 			GND
 * 	White 			GPIO Dir3
 * 	Green 			GPIO Dir2
 *
 * Note: Optical Encoder is LPD3806-600BM-G5-24C This encoder requires an open collector pull up resistor.
 * Note: GPIO_PULLUP is set in HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder) of
 * stm32f4xx_hal_msp.c Line 217
 *
 * Initial Motor Speed Profiles, Torque Current, and Overcurrent Thresholds set in l6474_target_config.h
 *
 *
 *
 */




/*
 * System Configuration Parameters
 *
 * ENABLE_SUSPENDED_PENDULUM_CONTROL is set to 0 for Inverted Pendulum and
 * 									 set to 1 for Suspended Pendulum
 * ENABLE_DUAL_PID is set to 1 to enable control action
 *
 * High Speed 2 millisecond Control Loop delay, 500 Hz Cycle System Configuration
 *
 * Motor Speed Profile Configurations are:
 *
 *						MAX_ACCEL: 3000; 	MAX_DECEL 3000
 * High Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 500
 * Medium Speed Mode:	MAX_SPEED: 1000; 	MIN_SPEED 300
 * Low Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 200
 * Suspended Mode: 		MAX_SPEED: 1000; 	MIN_SPEED 200
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "edukit_system.h"
//#include "control_loop.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>


/*
 * Motor Interface Data Structure
 *
 * Note that this is not required by default since Motor Profile is included
 * from l6474_target_config.h
 *
 * Note: This application is based on usage of l6474_target_config.h header
 * file for initialization of configuration of L6474
 */
L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};

 /* CMSIS */
#define ARM_MATH_CM4

/*
 * Apply acceleration
 */
#define PWM_COUNT_SAFETY_MARGIN 2
#define MAXIMUM_ACCELERATION 131071
#define MAXIMUM_DECELERATION 131071
#define MAXIMUM_SPEED 131071

volatile uint16_t gLastError;

float target_velocity_prescaled;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];

/// PWM period variables used by step interrupt
volatile uint32_t desired_pwm_period;
volatile uint32_t current_pwm_period;

/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
// Serial interface variables
static unsigned rxReaderHead = 0;

#define DATA_FRAME_SIZE 10
/*
 * Status response data layout:
 *        byte 0: command ID
 *        byte 1: device ID
 *        byte 2: error code (0 == OK)
 *        byte 3: total response size (13)
 *        byte 4: motorState_t
 *        bytes 5-8: motor position (int32_t)
 *        bytes 9-12: encoder position (uint32_t)
 */
#define STATUS_RESPONSE_SIZE 13
/*
 * Command response data layout:
 *        byte 0: command ID
 *        byte 1: device ID
 *        byte 2: error code (0 == OK)
 *        bytes 3-6: up to 4 bytes for return value
 */
#define COMMAND_RESPONSE_SIZE 7

enum CommandError {
	CMDERR_OK = 0U,
	CMDERR_INVALID_COMMAND = 1U,
	CMDERR_INVALID_DEVICE = 2U
};

/* Private function prototypes -----------------------------------------------*/
void MX_USART2_UART_Init(void);
void MX_TIM3_Init(void);
void MyFlagInterruptHandler(void);
void Main_StepClockHandler();
void apply_acceleration(float acc, float t_period);

int Delay_Pulse(){
	return desired_pwm_period == UINT32_MAX;
}

/*
 * PWM pulse (step) interrupt
 */
void Main_StepClockHandler() {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t desired_pwm_period_local = desired_pwm_period;

	if (desired_pwm_period_local != 0) {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}
}

/**
 * @brief Apply acceleration to the motor during real-time (RT) control.
 * @note: new_speed = old_speed + (acc * t_period)
 *
 * @param acc Acceleration to be applied.
 * @param t_period Period of one RT execution cycle.
 */
void apply_acceleration(float acc, float t_period) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t current_pwm_period_local = current_pwm_period;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	motorDir_t old_dir = target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (old_dir == FORWARD) {
		if (acc > MAXIMUM_ACCELERATION) {
			acc = MAXIMUM_ACCELERATION;
		} else if (acc < -MAXIMUM_DECELERATION) {
			acc = -MAXIMUM_DECELERATION;
		}
	} else {
		if (acc < -MAXIMUM_ACCELERATION) {
			acc = -MAXIMUM_ACCELERATION;
		} else if (acc > MAXIMUM_DECELERATION) {
			acc = MAXIMUM_DECELERATION;
		}	
	}

	target_velocity_prescaled += L6474_Board_Pwm1PrescaleFreq(acc) * t_period;
	motorDir_t new_dir = target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (target_velocity_prescaled > L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	} else if (target_velocity_prescaled < -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	}

	float speed_prescaled;
	if (new_dir == FORWARD) {
		speed_prescaled = target_velocity_prescaled;
	} else {
		speed_prescaled = target_velocity_prescaled * -1;
		if (speed_prescaled == 0) speed_prescaled = 0; // convert negative 0 to positive 0
	}


	uint32_t effective_pwm_period = desired_pwm_period_local;

	float desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / speed_prescaled);
	if (!(desired_pwm_period_float < 4294967296.0f)) {
		desired_pwm_period_local = UINT32_MAX;
	} else {
		desired_pwm_period_local = (uint32_t)(desired_pwm_period_float);
	}

	if (old_dir != new_dir) {
		L6474_Board_SetDirectionGpio(0, new_dir);
	}

	if (current_pwm_period_local != 0) {
		uint32_t pwm_count = L6474_Board_Pwm1GetCounter();
		uint32_t pwm_time_left = current_pwm_period_local - pwm_count;
		if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
			if (old_dir != new_dir) {
				// pwm_time_left = effective_pwm_period - pwm_time_left; // One method for assignment of PWM period during switching directions. This has the effect of additional discrete step noise.
				pwm_time_left = effective_pwm_period; // Second method for assignment of PWM period during switching directions. This shows reduced discrete step noise.
			}

			uint32_t new_pwm_time_left = ((uint64_t) pwm_time_left * desired_pwm_period_local) / effective_pwm_period;
			if (new_pwm_time_left != pwm_time_left) {
				if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
					new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
				}
				current_pwm_period_local = pwm_count + new_pwm_time_left;
				if (current_pwm_period_local < pwm_count) {
					current_pwm_period_local = UINT32_MAX;
				}

				L6474_Board_Pwm1SetPeriod(current_pwm_period_local);
				current_pwm_period = current_pwm_period_local;
			}
		}
	} else {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}

	desired_pwm_period = desired_pwm_period_local;

}

/**
 * @brief Frame has the following structure:
 *        byte 0: requested command ID
 *        byte 1: device ID
 *        bytes 2-9: command parameters
 */
struct DataFrame {
	uint8_t data[DATA_FRAME_SIZE];
};

/**
 * @brief Response has the following structure:
 *        byte 0: command ID
 *        byte 1: device ID
 *        byte 2: error code (0 == OK)
 *        bytes 3-6: command return values
 *        or
 *        bytes 3-12: status
 */
struct ResponseData {
	uint8_t data[STATUS_RESPONSE_SIZE];
};

void serialize_uint16(uint16_t value, uint8_t* dest) {
	dest[0] = (uint8_t)value;
	dest[1] = (uint8_t)(value >> 8);
}

void serialize_uint32(uint32_t value, uint8_t* dest) {
	dest[0] = (uint8_t)value;
	dest[1] = (uint8_t)(value >> 8);
	dest[2] = (uint8_t)(value >> 16);
	dest[3] = (uint8_t)(value >> 24);
}

void serialize_int32(int32_t value, uint8_t* dest) {
	uint32_t unsignedValue;
	memcpy(&unsignedValue, &value, sizeof(int32_t));
	serialize_uint32(unsignedValue, dest);
}

void serialize_float(float value, uint8_t* dest) {
	uint32_t unsignedValue;
	memcpy(&unsignedValue, &value, sizeof(float));
	serialize_uint32(unsignedValue, dest);
}

uint16_t deserialize_uint16(const uint8_t* data) {
	uint16_t value;
	value = data[0];
	value |= (data[1] << 8);
	return value;
}

uint32_t deserialize_uint32(const uint8_t* data) {
	uint32_t value;
	value = data[0];
	value |= (data[1] << 8);
	value |= (data[2] << 16);
	value |= (data[3] << 24);
	return value;
}

int32_t deserialize_int32(const uint8_t* data) {
	uint32_t usignedValue = deserialize_uint32(data);
	int32_t value;
	memcpy(&value, &usignedValue, sizeof(uint32_t));
	return value;
}

float deserialize_float(const uint8_t* data) {
	uint32_t usignedValue = deserialize_uint32(data);
	float value;
	memcpy(&value, &usignedValue, sizeof(uint32_t));
	return value;
}

/**
 * @brief Initialize the system (encoder, motor, serial interface...).
 */
void init_system(void) {
	// Initialize and enable cycle counter
	ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	DWT->CTRL |= 1; 		// at address 0xE0001000
	DWT->CYCCNT = 0; 		// at address 0xE0001004

	// STM32xx HAL library initialization
	if (HAL_Init() != HAL_OK) {
		Error_Handler(0);
	}

	// Configure the system clock
	SystemClock_Config();

	// Set the L6474 library to use 1 device
	if (!BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1)) {
		Error_Handler(0);
	}
	// Initialize Motor Control Library
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);

	// Initialize encoder
	MX_TIM3_Init();
	HAL_Delay(10);

	// Initialize UART communication port
	MX_USART2_UART_Init();
	// Start DMA just once because it's configured in "circular" mode
	if (HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE) != HAL_OK) {
		Error_Handler(0);
	}

	// Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	// Attach the function Error_Handler (defined below) to the error Handler
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	// Start encoder
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/**
 * @brief Read a single frame from serial interface.
 *
 * @param frame Received data. (Only written when function returns TRUE).
 * @return TRUE - frame was received and read. FALSE - no data available to read.
 */
bool read_frame(struct DataFrame* frame) {
	uint32_t rxWriterHead = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

	// Get number of available not yet handled bytes in the buffer.
	uint16_t newBytes;
	if (rxWriterHead >= rxReaderHead) {
		newBytes = rxWriterHead - rxReaderHead;
	} else {
		newBytes = UART_RX_BUFFER_SIZE + rxWriterHead - rxReaderHead;
	}

	if (newBytes < DATA_FRAME_SIZE) {
		// Still waiting for the whole frame.
		return FALSE;
	}

	// Copy frame data and move the read head.
	for (unsigned i = 0; i < DATA_FRAME_SIZE; ++i) {
		frame->data[i] = RxBuffer[rxReaderHead];
		if (++rxReaderHead >= UART_RX_BUFFER_SIZE) {
			rxReaderHead = 0;
		}
	}

	return TRUE;
}

/**
 * @brief Parse and execute command.
 *
 * @param frame Frame containing command data.
 * @return Response to be returned to the called.
 */
struct ResponseData handle_command(const struct DataFrame* frame) {

	uint8_t commandId = frame->data[0];
	uint8_t deviceId = frame->data[1];
	const uint8_t* params = frame->data + 2;

	struct ResponseData response;
	response.data[0] = commandId;
	response.data[1] = deviceId;
	response.data[2] = CMDERR_OK;
	uint8_t* commandReturnValue = response.data + 3;

	// Make sure device ID is valid, if command expects device ID.
	if (deviceId > BSP_MotorControl_GetNbDevices()) {
		switch (commandId) {
			case 6: case 7: case 8: case 9: case 11: case 12: case 13: case 14: case 15:
			case 16: case 17: case 18: case 19: case 21: case 22: case 23: case 24: case 25:
			case 26: case 27: case 28: case 30: case 31: case 32: case 33: case 34: case 35:
			case 36: case 37: case 40: case 41: case 46: case 68: case 69: case 72: case 73:
			case 82: case 83: case 252: case 253: case 254:
				response.data[2] = CMDERR_INVALID_DEVICE;
				return response;
		}
	}

	// Command IDs are ordered in the same way as they appear in file motor.h in definition
	// of motorDrv_t struct.
	switch (commandId) {
		case 6: {
			uint16_t acc = BSP_MotorControl_GetAcceleration(deviceId);
			serialize_uint16(acc, commandReturnValue);
		} break;
		case 7: {
			uint16_t speed = BSP_MotorControl_GetCurrentSpeed(deviceId);
			serialize_uint16(speed, commandReturnValue);
		} break;
		case 8: {
			uint16_t deceleration = BSP_MotorControl_GetDeceleration(deviceId);
			serialize_uint16(deceleration, commandReturnValue);
		} break;
		case 9: {
			motorState_t state = BSP_MotorControl_GetDeviceState(deviceId);
			commandReturnValue[0] = state;
		} break;
		case 10: {
			uint32_t version = BSP_MotorControl_GetFwVersion();
			serialize_uint32(version, commandReturnValue);
		} break;
		case 11: {
			int32_t mark = BSP_MotorControl_GetMark(deviceId);
			serialize_int32(mark, commandReturnValue);
		} break;
		case 12: {
			uint16_t maxSpeed = BSP_MotorControl_GetMaxSpeed(deviceId);
			serialize_uint16(maxSpeed, commandReturnValue);
		} break;
		case 13: {
			uint16_t minSpeed = BSP_MotorControl_GetMinSpeed(deviceId);
			serialize_uint16(minSpeed, commandReturnValue);
		} break;
		case 14: {
			int32_t position = BSP_MotorControl_GetPosition(deviceId);
			serialize_int32(position, commandReturnValue);
		} break;
		case 15:
			BSP_MotorControl_GoHome(deviceId);
			break;
		case 16:
			BSP_MotorControl_GoMark(deviceId);
			break;
		case 17:
			BSP_MotorControl_GoTo(deviceId, deserialize_int32(params));
			break;
		case 18:
			BSP_MotorControl_HardStop(deviceId);
			break;
		case 19:
			BSP_MotorControl_Move(deviceId, params[0], deserialize_int32(params + 1));
			break;
		case 20:
			BSP_MotorControl_ResetAllDevices();
			break;
		case 21:
			BSP_MotorControl_Run(deviceId, params[0]);
			break;
		case 22: {
			bool ret = BSP_MotorControl_SetAcceleration(deviceId, deserialize_uint16(params));
			commandReturnValue[0] = ret;
		} break;
		case 23: {
			bool ret = BSP_MotorControl_SetDeceleration(deviceId, deserialize_uint16(params));
			commandReturnValue[0] = ret;
		} break;
		case 24:
			BSP_MotorControl_SetHome(deviceId, deserialize_int32(params));
			break;
		case 25:
			BSP_MotorControl_SetMark(deviceId, deserialize_int32(params));
			break;
		case 26: {
			bool ret = BSP_MotorControl_SetMaxSpeed(deviceId, deserialize_uint16(params));
			commandReturnValue[0] = ret;
		} break;
		case 27: {
			bool ret = BSP_MotorControl_SetMinSpeed(deviceId, deserialize_uint16(params));
			commandReturnValue[0] = ret;
		} break;
		case 28: {
			bool ret = BSP_MotorControl_SoftStop(deviceId);
			commandReturnValue[0] = ret;
		} break;
		case 30:
			BSP_MotorControl_WaitWhileActive(deviceId);
			break;
		case 31:
			BSP_MotorControl_CmdDisable(deviceId);
			break;
		case 32:
			BSP_MotorControl_CmdEnable(deviceId);
			break;
		case 33: {
			uint32_t param = BSP_MotorControl_CmdGetParam(deviceId, deserialize_uint32(params));
			serialize_uint32(param, commandReturnValue);
		} break;
		case 34: {
			uint16_t status = BSP_MotorControl_CmdGetStatus(deviceId);
			serialize_uint16(status, commandReturnValue);
		} break;
		case 35:
			BSP_MotorControl_CmdNop(deviceId);
			break;
		case 36:
			BSP_MotorControl_CmdSetParam(deviceId, deserialize_uint32(params),
					deserialize_uint32(params + 4));
			break;
		case 37: {
			uint16_t status = BSP_MotorControl_ReadStatusRegister(deviceId);
			serialize_uint16(status, commandReturnValue);
		} break;
		case 38:
			BSP_MotorControl_ReleaseReset(deviceId);
			break;
		case 39:
			BSP_MotorControl_Reset(deviceId);
			break;
		case 40: {
			bool ret = BSP_MotorControl_SelectStepMode(deviceId, params[0]);
			commandReturnValue[0] = ret;
		} break;
		case 41:
			BSP_MotorControl_SetDirection(deviceId, params[0]);
			break;
		case 46:
			BSP_MotorControl_CmdHardHiZ(deviceId);
			break;
		case 55: {
			uint8_t numOfDev = BSP_MotorControl_GetNbDevices();
			commandReturnValue[0] = numOfDev;
		} break;
		case 68:
			BSP_MotorControl_SetStopMode(deviceId, params[0]);
			break;
		case 69: {
			motorStopMode_t stopMode = BSP_MotorControl_GetStopMode(deviceId);
			commandReturnValue[0] = stopMode;
		} break;
		case 72: {
			motorStepMode_t stepMode = BSP_MotorControl_GetStepMode(deviceId);
			commandReturnValue[0] = stepMode;
		} break;
		case 73: {
			motorDir_t direction = BSP_MotorControl_GetDirection(deviceId);
			commandReturnValue[0] = direction;
		} break;
		case 82: {
			bool ret = BSP_MotorControl_SetAnalogValue(deviceId, deserialize_uint32(params),
					deserialize_float(params + 4));
			commandReturnValue[0] = ret;
		} break;
		case 83: {
			float value = BSP_MotorControl_GetAnalogValue(deviceId, deserialize_uint32(params));
			serialize_float(value, commandReturnValue);
		} break;
		case 252: {
			// Custom command to end RT control of the motor.
			BSP_MotorControl_GoTo(deviceId, 0);
			bool ret = BSP_MotorControl_SoftStop(deviceId);
			NVIC_SystemReset();
			desired_pwm_period = 0;
			current_pwm_period = 0;
			commandReturnValue[0] = ret;
		} break;
		case 253: {
			// Custom command for reading the system status.
			int32_t position = BSP_MotorControl_GetPosition(deviceId);
			uint32_t encoder = __HAL_TIM_GET_COUNTER(&htim3);
			// Set size of the response.
			commandReturnValue[0] = STATUS_RESPONSE_SIZE;
			commandReturnValue[1] = BSP_MotorControl_GetDeviceState(deviceId);
			serialize_int32(position, commandReturnValue + 2);
			serialize_uint32(encoder, commandReturnValue + 6);
		} break;
		case 254:
			// Custom command for RT control of the motor.
			apply_acceleration(deserialize_float(params), deserialize_float(params + 4));
			break;
		default:
			response.data[2] = CMDERR_INVALID_COMMAND;
			break;
	}
	return response;
}

void send_response(struct ResponseData* response) {
	if (response->data[0] == 254) {
		// No response is expected for RT control command.
		return;
	}
	if (response->data[0] == 253) {
		HAL_UART_Transmit(&huart2, response->data, STATUS_RESPONSE_SIZE, HAL_MAX_DELAY);
		return;
	}

	// Send response for standard/other API calls.
	HAL_UART_Transmit(&huart2, response->data, COMMAND_RESPONSE_SIZE, HAL_MAX_DELAY);
}

int main(void) {
	init_system();

	struct DataFrame frame;
	while(TRUE) {
		if (read_frame(&frame)) {
			struct ResponseData response = handle_command(&frame);
			send_response(&response);
		}
	}

	return 0;
}

/* TIM3 init function */
void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}
}

/* USART2 init function */
void MX_USART2_UART_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
							;

	huart2.Instance = USART2;
	huart2.Init.BaudRate = SAMPLE_BAUD_RATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler(0);
	}

	/* USART2 RX DMA Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
		Error_Handler(0);
	}
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void) {
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
		// HIZ state
		// Action to be customized
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
		// Forward direction is set
		// Action to be customized
	} else {
		// Backward direction is set
		// Action to be customized
	}

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD)
			== L6474_STATUS_NOTPERF_CMD) {
		// Command received by SPI can't be performed
		// Action to be customized
	}

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
		//command received by SPI does not exist
		// Action to be customized
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0) {
		//undervoltage lock-out
		// Action to be customized
	}

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
		//thermal warning threshold is reached
		// Action to be customized
	}

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
		//thermal shut down threshold is reached
		// Action to be customized
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0) {
		//overcurrent detection
		// Action to be customized
	}
}

/**
 * @brief  This function is executed in event of error occurrence.
 * @param  error number of the error event
 * @retval None
 */
void Error_Handler(uint16_t error) {
	/* Backup error number */
	gLastError = error;

	/* Infinite loop */
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



