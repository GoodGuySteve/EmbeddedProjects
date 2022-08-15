/* The goal of a PID controller is to output a signal using input from a related signal to achieve
   a targeted value in a manner that is responsive, accurate, and stable. Since the input signal
   is often related to the environment, this means that it may be subject to fluctuations 
   outside the control of the program and its output signal.

   A PID controller achieves this by utilizing three functions:
   - Integral (I) - A value which adjusts towards the set point based on accumulated error.
   The accumulation makes this slow, but it is accurate when it converges.
   - Proportion (P) - A value which adjusts towards the set point based on instantaneous error. 
   This improves the responsiveness of the slow integral function.
   - Derivative (D) - A value which compares error values and counterracts the direction of the
   error. This can dampen oscillations of the other two signals, but it can also magnify noise
   from the environment.

   In a generic controller like this, all three of the PID values can be tuned, and the output
   value given is the combination of all three functions.
*/

class PidController {
	public:
		PidController();
		PidController& operator=(const PidController& other);
		~PidController();
		PidController(double proportionWeight, double integralWeight, double derivativeWeight,
			double setPoint);

		/* Primary functions for using the PID controller */

		/* Given an input value, generate an output value intended to adjust the input value 
		towards the setPoint. The input signal should therefore be correlated somehow with 
		the output of the PidController. */
		double nextOutput(double input);
		/* Get the value of the input signal the PID controller is attempting to achieve. */
		double getSetPoint(void);
		/* Set a new value for the PID controller to attempt to move the input signal towards.
		This can be adjusted on the fly. */
		void setSetPoint(double setPoint);

		/* Utility functions. PID weights are not typically changed on the fly but is allowed */
		double getProportionWeight(void);
		double getIntegralWeight(void);
		double getDerivativeWeight(void);
		void setPid(double proportionWeight, double integralWeight, double derivativeWeight);

	private:
		double proportionWeight;
		double integralWeight;
		double derivativeWeight;
		double setPoint;
		double prevError;
		double cumulativeIntegral;
        
};