/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	Project
*****************************************************************
**  Brief
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**	Test Four VNH7040
**	Give a ramp each
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**
****************************************************************/

/****************************************************************
**	USED PINS
****************************************************************
**	VNH7040
**				|	DRV0	|	DRV1	|	DRV2	|	DRV3	|	VNH7040
**	-------------------------------------------------------------------------
**	uC_SEN		|	PF0		|	PF0		|	PF0		|	PF0		|	SENSE ENABLE
**	uC_DIAG		|	PF1		|	PF1		|	PF1		|	PF1		|	SEL1
**	uC_PWM		|	PA2,B20	|	PA3,B21	|	PB4,B22	|	PB5,B23	|	PWM
**	uC_CTRLA	|	PA4		|	PA6		|	PB2		|	PD6		|	INA, SEL0
**	uC_CTRLB	|	PA5		|	PA7		|	PB3		|	PD7		|	INB

****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**
****************************************************************/

/****************************************************************
**	DEFINES
****************************************************************/

#define EVER (;;)

/****************************************************************
**	INCLUDES
****************************************************************/

//Pin Configuration
//PF5: Embedded LED0

#include "global.h"

/****************************************************************
** FUNCTION PROTOTYPES
****************************************************************/

//Set direction and speed setting of the VNH7040 controlled motor
extern void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed );

/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

volatile Isr_flags g_isr_flags;

/****************************************************************************
**  Function
**  main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details test the declaration of a lambda method
/***************************************************************************/

int main(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//system tick prescaler
	uint8_t pre = 0;
		//! Motor Test
	//index of the motor under test
	uint8_t motor_index = 0;
	//Speed ramp. false = accelerate | true = decelerate
	bool f_ramp = false;
	//Direction. false =  clockwise | true = counterclockwise
	bool f_dir = false;
	//Speed counter
	uint8_t speed = 0;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//! Initialize AT4809 internal peripherals
	init();
		//!	Initialize VNH7040
	//Enable sense output
	SET_BIT_VALUE( PORTF.OUT, 0, true );
	//Diagnostic mode OFF
	SET_BIT_VALUE( PORTF.OUT, 1, false );
	
	//Set direction stop
	SET_BIT_VALUE( PORTA.OUT, 4, false );	//CTRLA
	SET_BIT_VALUE( PORTA.OUT, 5, false );	//CTRLB
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main loop
	for EVER
	{
		//If: System Tick 500Hz
		if (g_isr_flags.system_tick == 1)
		{
			//Clear system tick
			g_isr_flags.system_tick = 0;
			
			//----------------------------------------------------------------
			//	LED BLINK
			//----------------------------------------------------------------
			
			//If prescaler has reset
			if (pre == 0)
			{
				//Toggle PF5.
				SET_BIT( PORTF.OUTTGL, 5 );	
			}
			//Increment prescaler and reset if it exceeds the TOP. 2Hz
			pre = AT_TOP_INC( pre, 249);
			
			//----------------------------------------------------------------
			//	MOTOR CONTROL
			//----------------------------------------------------------------
			
			//If increase speed
			if (f_ramp == false)
			{
				//increase speed
				speed++;
				//if: speed cap
				if (speed >= (uint8_t)0xff)
				{
					//now decrease speed
					f_ramp = true;
				}
			}
			//If decrease speed
			else
			{
				//decrease speed
				speed--;
				//If: speed cap
				if (speed <= (uint8_t)0x00)
				{
					//Now increase speed
					f_ramp = false;
					//Invert motor direction
					f_dir = !f_dir;
					//If I'm going forward again
					if (f_dir == false)
					{
						//change motor under test
						motor_index = AT_TOP_INC( motor_index, 3);
					}
				}
			}
			//Set direction and speed setting of the VNH7040 controlled motor
			set_vnh7040_speed( motor_index, f_dir, speed );
		}	//End If: System Tick
	}	//End: Main loop

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return 0;
}	//end: main

/****************************************************************************
**  Function
**  set_vnh7040_speed
****************************************************************************/
//! @return void |
//! @param index	| index of the motor to be controlled. 0 to 3
//! @param f_dir	| direction of rotation of the motor
//! @param speed	| Speed of the motor
//! @brief Set direction and speed setting of the VNH7040 controlled motor
//! @details 
//!
/***************************************************************************/

void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Driver 0
	if (index == 0)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x10);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x20);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB0.CCMPH = speed;	
	}
	//Driver 1
	else if (index == 1)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB1.CCMPH = speed;
	}
	//Driver 2
	else if (index == 2)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x04);
		}
		else
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x08);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB2.CCMPH = speed;
	}
	//Driver 3
	else if (index == 3)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB3.CCMPH = speed;
	}
	//Default case
	else
	{
		//Driver index not installed.
		//Do nothing
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: 
