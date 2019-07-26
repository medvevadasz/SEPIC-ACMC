/* ***************************************************************************************
 * z-Domain Control Loop Designer Version 0.9.0.60.
 * ***************************************************************************************
 * 2p2z compensation filter coefficients derived for following operating conditions:
 * ***************************************************************************************
 *
 * 	Controller Type:	2P2Z - Basic Current Mode Compensator
 * 	Sampling Frequency:	350000 Hz 
 * 	Fixed Point Format:	15
 * 	Scaling Mode:		2 - Single Bit-Shift with Output Factor Scaling
 * 	Input Gain:			0.148
 * 
 * ***************************************************************************************/

#include "c2p2z_sepic_current.h"

/* ***************************************************************************************
 * Data Arrays:
 * The cNPNZ_t data structure contains a pointer to derived coefficients in X-space and
 * other pointers to controller and error history in Y-space.
 * This source file declares the default parameters of the z-domain compensation filter.
 * These declarations are made publicly accessible through defines in c2p2z_sepic_current.h
 * ***************************************************************************************/

	volatile C2P2Z_SEPIC_CURRENT_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c2p2z_sepic_current_coefficients; // A/B-Coefficients 
	volatile uint16_t c2p2z_sepic_current_ACoefficients_size = (sizeof(c2p2z_sepic_current_coefficients.ACoefficients)/sizeof(c2p2z_sepic_current_coefficients.ACoefficients[0])); // A-coefficient array size
	volatile uint16_t c2p2z_sepic_current_BCoefficients_size = (sizeof(c2p2z_sepic_current_coefficients.BCoefficients)/sizeof(c2p2z_sepic_current_coefficients.BCoefficients[0])); // B-coefficient array size

	volatile C2P2Z_SEPIC_CURRENT_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c2p2z_sepic_current_histories; // Control/Error Histories 
	volatile uint16_t c2p2z_sepic_current_ControlHistory_size = (sizeof(c2p2z_sepic_current_histories.ControlHistory)/sizeof(c2p2z_sepic_current_histories.ControlHistory[0])); // Control history array size
	volatile uint16_t c2p2z_sepic_current_ErrorHistory_size = (sizeof(c2p2z_sepic_current_histories.ErrorHistory)/sizeof(c2p2z_sepic_current_histories.ErrorHistory[0])); // Error history array size

/* ***************************************************************************************
 * 	Pole&Zero Placement:
 * ***************************************************************************************
 *
 * 	fP0:	300 Hz 
 * 	fP1:	60000 Hz 
 * 	fZ1:	300 Hz 
 *
 * ***************************************************************************************
 * 	Filter Coefficients and Parameters:
 * ***************************************************************************************/

	volatile fractional c2p2z_sepic_current_ACoefficients [2] = 
	{
		0x4629,	// Coefficient A1 will be multiplied with controller output u(n-1)
		0xEFD1	// Coefficient A2 will be multiplied with controller output u(n-2)
	};

	volatile fractional c2p2z_sepic_current_BCoefficients [3] = 
	{
		0x7FFF,	// Coefficient B0 will be multiplied with error input e(n)
		0x00B0,	// Coefficient B1 will be multiplied with error input e(n-1)
		0x80B1	// Coefficient B2 will be multiplied with error input e(n-2)
	};


	volatile int16_t c2p2z_sepic_current_pre_scaler = 3;
	volatile int16_t c2p2z_sepic_current_post_shift_A = -2;
	volatile int16_t c2p2z_sepic_current_post_shift_B = 0;
	volatile fractional c2p2z_sepic_current_post_scaler = 0x4BE5;

	volatile cNPNZ16b_t c2p2z_sepic_current; // user-controller data object

/* ***************************************************************************************/

uint16_t c2p2z_sepic_current_Init(void)
{
	volatile uint16_t i = 0;

	// Initialize controller data structure at runtime with pre-defined default values
	c2p2z_sepic_current.status.flags = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))

	c2p2z_sepic_current.ptrACoefficients = &c2p2z_sepic_current_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
	c2p2z_sepic_current.ptrBCoefficients = &c2p2z_sepic_current_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
	c2p2z_sepic_current.ptrControlHistory = &c2p2z_sepic_current_histories.ControlHistory[0]; // initialize pointer to control history array
	c2p2z_sepic_current.ptrErrorHistory = &c2p2z_sepic_current_histories.ErrorHistory[0]; // initialize pointer to error history array
	c2p2z_sepic_current.normPostShiftA = c2p2z_sepic_current_post_shift_A; // initialize A-coefficients/single bit-shift scaler
	c2p2z_sepic_current.normPostShiftB = c2p2z_sepic_current_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
	c2p2z_sepic_current.normPostScaler = c2p2z_sepic_current_post_scaler; // initialize control output value normalization scaling factor
	c2p2z_sepic_current.normPreShift = c2p2z_sepic_current_pre_scaler; // initialize A-coefficients/single bit-shift scaler

	c2p2z_sepic_current.ACoefficientsArraySize = c2p2z_sepic_current_ACoefficients_size; // initialize A-coefficients array size
	c2p2z_sepic_current.BCoefficientsArraySize = c2p2z_sepic_current_BCoefficients_size; // initialize A-coefficients array size
	c2p2z_sepic_current.ControlHistoryArraySize = c2p2z_sepic_current_ControlHistory_size; // initialize control history array size
	c2p2z_sepic_current.ErrorHistoryArraySize = c2p2z_sepic_current_ErrorHistory_size; // initialize error history array size


	// Load default set of A-coefficients from user RAM into X-Space controller A-array
	for(i=0; i<c2p2z_sepic_current.ACoefficientsArraySize; i++)
	{
		c2p2z_sepic_current_coefficients.ACoefficients[i] = c2p2z_sepic_current_ACoefficients[i];
	}

	// Load default set of B-coefficients from user RAM into X-Space controller B-array
	for(i=0; i<c2p2z_sepic_current.BCoefficientsArraySize; i++)
	{
		c2p2z_sepic_current_coefficients.BCoefficients[i] = c2p2z_sepic_current_BCoefficients[i];
	}

	// Clear error and control histories of the 2P2Z controller
	c2p2z_sepic_current_Reset(&c2p2z_sepic_current);

	return(1);
}


