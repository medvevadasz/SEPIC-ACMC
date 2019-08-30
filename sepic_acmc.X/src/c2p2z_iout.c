/* ***************************************************************************************
 * z-Domain Control Loop Designer Version 0.9.0.60.
 * ***************************************************************************************
 * 2p2z compensation filter coefficients derived for following operating conditions:
 * ***************************************************************************************
 *
 * 	Controller Type:	2P2Z - Basic Current Mode Compensator
 * 	Sampling Frequency:	350000 Hz 
 * 	Fixed Point Format:	15
 * 	Scaling Mode:		4 - Fast Floating Point Coefficient Scaling
 * 	Input Gain:			2.5
 * 
 * ***************************************************************************************/

#include "c2p2z_iout.h"

/* ***************************************************************************************
 * Data Arrays:
 * The cNPNZ_t data structure contains a pointer to derived coefficients in X-space and
 * other pointers to controller and error history in Y-space.
 * This source file declares the default parameters of the z-domain compensation filter.
 * These declarations are made publicly accessible through defines in c2p2z_iout.h
 * ***************************************************************************************/

	volatile C2P2Z_IOUT_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c2p2z_iout_coefficients; // A/B-Coefficients 
	volatile uint16_t c2p2z_iout_ACoefficients_size = (sizeof(c2p2z_iout_coefficients.ACoefficients)/sizeof(c2p2z_iout_coefficients.ACoefficients[0])); // A-coefficient array size
	volatile uint16_t c2p2z_iout_BCoefficients_size = (sizeof(c2p2z_iout_coefficients.BCoefficients)/sizeof(c2p2z_iout_coefficients.BCoefficients[0])); // B-coefficient array size

	volatile C2P2Z_IOUT_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c2p2z_iout_histories; // Control/Error Histories 
	volatile uint16_t c2p2z_iout_ControlHistory_size = (sizeof(c2p2z_iout_histories.ControlHistory)/sizeof(c2p2z_iout_histories.ControlHistory[0])); // Control history array size
	volatile uint16_t c2p2z_iout_ErrorHistory_size = (sizeof(c2p2z_iout_histories.ErrorHistory)/sizeof(c2p2z_iout_histories.ErrorHistory[0])); // Error history array size

/* ***************************************************************************************
 * 	Pole&Zero Placement:
 * ***************************************************************************************
 *
 * 	fP0:	5 Hz 
 * 	fP1:	10000 Hz 
 * 	fZ1:	400 Hz 
 *
 * ***************************************************************************************
 * 	Filter Coefficients and Parameters:
 * ***************************************************************************************/

	volatile int32_t c2p2z_iout_ACoefficients [2] = 
	{
		0x7576FFFF,	// Coefficient A1 will be multiplied with controller output u(n-1)
		0x95160000	// Coefficient A2 will be multiplied with controller output u(n-2)
	};

	volatile int32_t c2p2z_iout_BCoefficients [3] = 
	{
		0x6C59000B,	// Coefficient B0 will be multiplied with error input e(n)
		0x633B0012,	// Coefficient B1 will be multiplied with error input e(n-1)
		0x946E000B	// Coefficient B2 will be multiplied with error input e(n-2)
	};


	volatile int16_t c2p2z_iout_pre_scaler = 3;
	volatile int16_t c2p2z_iout_post_shift_A = 0;
	volatile int16_t c2p2z_iout_post_shift_B = 0;
	volatile fractional c2p2z_iout_post_scaler = 0x0000;

	volatile cNPNZ16b_t c2p2z_iout; // user-controller data object

/* ***************************************************************************************/

uint16_t c2p2z_iout_Init(void)
{
	volatile uint16_t i = 0;

	// Initialize controller data structure at runtime with pre-defined default values
	c2p2z_iout.status.flags = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))

	c2p2z_iout.ptrACoefficients = &c2p2z_iout_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
	c2p2z_iout.ptrBCoefficients = &c2p2z_iout_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
	c2p2z_iout.ptrControlHistory = &c2p2z_iout_histories.ControlHistory[0]; // initialize pointer to control history array
	c2p2z_iout.ptrErrorHistory = &c2p2z_iout_histories.ErrorHistory[0]; // initialize pointer to error history array
	c2p2z_iout.normPostShiftA = c2p2z_iout_post_shift_A; // initialize A-coefficients/single bit-shift scaler
	c2p2z_iout.normPostShiftB = c2p2z_iout_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
	c2p2z_iout.normPostScaler = c2p2z_iout_post_scaler; // initialize control output value normalization scaling factor
	c2p2z_iout.normPreShift = c2p2z_iout_pre_scaler; // initialize A-coefficients/single bit-shift scaler

	c2p2z_iout.ACoefficientsArraySize = c2p2z_iout_ACoefficients_size; // initialize A-coefficients array size
	c2p2z_iout.BCoefficientsArraySize = c2p2z_iout_BCoefficients_size; // initialize A-coefficients array size
	c2p2z_iout.ControlHistoryArraySize = c2p2z_iout_ControlHistory_size; // initialize control history array size
	c2p2z_iout.ErrorHistoryArraySize = c2p2z_iout_ErrorHistory_size; // initialize error history array size


	// Load default set of A-coefficients from user RAM into X-Space controller A-array
	for(i=0; i<c2p2z_iout.ACoefficientsArraySize; i++)
	{
		c2p2z_iout_coefficients.ACoefficients[i] = c2p2z_iout_ACoefficients[i];
	}

	// Load default set of B-coefficients from user RAM into X-Space controller B-array
	for(i=0; i<c2p2z_iout.BCoefficientsArraySize; i++)
	{
		c2p2z_iout_coefficients.BCoefficients[i] = c2p2z_iout_BCoefficients[i];
	}

	// Clear error and control histories of the 2P2Z controller
	c2p2z_iout_Reset(&c2p2z_iout);

	return(1);
}


