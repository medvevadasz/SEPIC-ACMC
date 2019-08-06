To run on SEPIC board, switch configuration to "SEPIC_wCKPIM"

Features (and lack of ...)

	-	This firmware serves to set the proper quoefficients for the inner (input)current loop compensator. The current loop is closed all the time. Voltage loop is not used.

	- 	During power-up sequencing offset voltage is measured at the output of the current sensor before any PWM activity takes place. This value is fed into the current compensator.
		Set point for the current loop is then gradually increased to 1A. In steady state the set point remains at 1A.
		
	- 	PWM duty cycle is limited to 200 ns minimum and 0.8*(1/350kHz) maximum. 

	- 	Output voltage is monitored only in SW. Fault event occurs when output voltage hits 20V. The system than falls back to standby mode, 
		where output voltage is constantly monitored. In case the output voltage remains in the normal operation range for 1000 ms, the firmware attempts to re-start the system.
		
	-	Set point vs. current feedback signal difference is not monitored.	

Settings

	"globals.h" contains all the relevant parameters that can be tweaked. The following parameters might be of most interest:

		#define SEPIC_IIN_SOFT_START_REFERENCE  1.0  //  [A] This is the value the ramp-up process converges to; Only for establishing current compensator characteristics 

		#define SEPIC_IIN_RS            0.05    // [Ohm]
		#define SEPIC_IIN_R1            1.0     // [kOhm]
		#define SEPIC_IIN_R2            4.7     // [kOhm]
		#define SEPIC_IIN_R3            18.0    // [kOhm]

		#define SEPIC_IIN_FB_GAIN           (float)((IINRS * IINR3/IINR1 * (IINR1 + IINR2)/(IINR1 + IINR3) ))
		~~~~~~~~~~~~~~~~

		#define SEPIC_VOUT_MAXIMUM          20.0    // Maximum output voltage in [V]
		#define SEPIC_VOUT_HYSTERESIS       1.0     // Output voltage protection hysteresis in [V]

		#define SEPIC_VOUT_R1           (2.0 * 2.87) // Upper voltage divider resistor in kOhm
		#define SEPIC_VOUT_R2           1.0          // Lower voltage divider resistor in kOhm

		#define SEPIC_VOUT_FB_GAIN      (float)((SEPIC_VOUT_R2) / (SEPIC_VOUT_R1 + SEPIC_VOUT_R2))

		#define SEPIC_FAULT_SHUT_DOWN_DELAY   1e-3          // shut down delay in [sec]
		#define SEPIC_FAULT_RECOVERY_DELAY    1000e-3       // recovery delay in [sec]

		~~~~~~~~~~~~~~~~

		#define SEPIC_POWER_ON_DELAY    500e-3      // power on delay in [sec]
		#define SEPIC_RAMP_PERIOD       20e-3         // ramp period in [sec] - Note this is just approximate value due to round-off error occuring when establishing ramp increment per 100us step
		#define SEPIC_POWER_GOOD_DELAY  100e-3        // power good in [sec]







