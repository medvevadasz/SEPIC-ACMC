To run on SEPIC board, switch configuration to "SEPIC_wCKPIM"

Features (and lack of ...)

	- 	This firmware is meant to implement average current mode current regulator. Inner loop is controlling
		average input current while outer loop is controlling output current.
		
	- 	ADC for input current measurement is triggered @ PG1DC/2 and result is read in the assigned ISR.
		The same ISR is used to read the output current. Interrupt is not enabled for the ADC assigned to the output current measurement.
	
	- 	Signal propagation delay is not accounted for yet when measuring average input current
	
	-	PWM duty cycle is limited to 200 ns minimum and 0.8*(1/350kHz) maximum
	
	- 	Output voltage is not monitored
	
	- 	Fault handling is not in place
	
		
	
Issues:

	- 	The software breaks in a MPY instruction in the outer current compensator when gain set > 0.97.
		Reason for this is not known yet. 
		Please check "iout_controller_update_issue_when_gain_greater_0.97.PNG".

State Machine:

	/*!SEPIC_SS_INIT
         * When SEPIC converter is in initialization mode, the basic 
         * peripherals needed for the operation of the power converter
         * will be set up. This step is only executed once. */
	
	/*!SEPIC_SS_LAUNCH_PER
         * Once all required peripherals have been initialized, the peripheral blocks and
         * control related software modules are enabled in a dedicated sequence. The PWM 
         * outputs and control loop execution, however, are still disabled and the power
         * supply will not start up yet. This step is only executed once and will complete by 
         * switching into STANDBY. */	
	
	/*!SEPIC_SS_STANDBY
         * This state is entered once all required peripherals have been launched and the 
         * power supply is waiting to be powered up. This is also the standard fall-back 
         * state after a fault/restart condition.
         * To get the power supply to start, all faults status bits need to be cleared,
         * the ADC has to run and produce data, the power controller has to be enabled 
         * and the status bit "sepic.status.flags.GO" has to be set.
         * 
         * Reference of the output current controller is hijacked to "sepic.soft_start.iout_reference" (0.25 A)
         * and the output is clamped to limit initial setpoint (0.2 A) for the inner current loop. 
         * 
         * Please note:
         * The data structure sepic.status.flags also offers a setting called auto_start.
         * When this bit is set, the 'enable' and 'GO' bits are set automatically and only
         * the 'adc_active' and 'fault_active' bits are checked. */
	
	/*!SEPIC_SS_CAL_IIN_OFFSET
         * In this state the the offset pertaining to the input current measurement is established
         * and fed into the input current compensator. Once the offset has been established, PWM output,
         * inner and outer current loops get enabled.  At the end of this phase, the state automatically 
         * switches to SEPIC_SS_WAIT_IOUT_SENSE mode. */       
		
	/*!SEPIC_SS_WAIT_IOUT_SENSE
         * In this state the inner current loop provides the load with limited amount of current
         * - set in STANDBY mode - to raise the output voltage above approximately 2.5 V, so that
         * the output current sensor produces valid data. Once 8 consecutive samples taken at the 
         * output are considered valid, the state machine switches to SEPIC_SS_INCR_CLAMP mode. */  

	/*!SEPIC_SS_INCR_CLAMP
         * Once this state is reached, upper output limit of the output current compensator providing
         * reference value to the inner current loop is increased step-by-step until the defined
         * maximum is reached, allowing the amount of controlled output current to flow  that is determined
         * by the value set in STANDBY mode.*/  

	/*!SEPIC_SS_RAMP_UP
         * During ramp up, the outer loop control reference is incremented. 
         * Once the 'private' reference of the soft-start data structure equals the
         * reference level set in sepic.data.v_ref, the ramp-up period ends and the state machine 
         * automatically switches to POWER GOOD DELAY mode */     		 
	
	/*!SEPIC_SS_PWR_GOOD_DELAY
         * POWER GOOD DELAY is just like POWER ON DELAY a state in which the soft-start counter
         * is counting call intervals until the user defined period has expired. Then the state 
         * machine automatically switches to COMPLETE mode */  
		 
	/*!SEPIC_SS_COMPLETE
         * The COMPLETE phase is the default state of the power controller. Once entered, only a FAULT
         * condition or external modifications of the soft-start phase can trigger a change of state. */  