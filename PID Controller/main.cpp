/*
    Code for testing the PID Controller class.
*/
#include "PidController.h"
#include <iostream>
#include <sstream>
#include <random>
#include <ctime>
#include <queue>

using namespace std;

const int numSteps = 500;

mt19937 rng;

/*------------------------------
  Process functions - for simulating different effects of the controller's output
  on the input signal
  ------------------------------*/

// Simply take the output and scale it down before it is fed back into the controller
double scale(double pidOutput) {
	return pidOutput / 10000.0;
}

// Applies a one-sided bias to the output
double offset(double pidOutput) {
	return scale(pidOutput + 10000);
}

queue<double> delayQueue;
// Delays the effects of the PID controller by a few steps. Note that delayQueue must 
// be initialized with initial delay values for this function to delay as expected
double delay(double pidOutput) {
	// Save the latest value, use the oldest
	delayQueue.push(pidOutput);
	double value = delayQueue.front();
	delayQueue.pop();

	return scale(value);
}

// Adds random noise to the signal
double noise(double pidOutput) {
	uniform_real_distribution<double> distribution(0.0, 1.0);
	return scale(pidOutput + distribution(rng)*100 - 50);
}

/*------------------------------
  Test functions - for defining different tests to run on the PidController class.
  This also demonstrates how the class is used. 
  ------------------------------*/

// Tests whether a the input converges on the setPoint after a set number of steps
bool testConvergence(double setPoint, double proportionWeight, double integralWeight,
	double derivativeWeight, double (*processFunction)(double), string functionName,
	bool shouldConverge = true) {

	PidController pid;
	pid.setPid(proportionWeight, integralWeight, derivativeWeight);
	pid.setSetPoint(setPoint);

	bool isSuccess = false;

	double lastOutput = 0;
	double nextInput = processFunction(lastOutput);
	for (int i = 0; i < 500; ++i) {
		lastOutput = pid.nextOutput(nextInput);
		nextInput = processFunction(lastOutput); 
#if defined(VERBOSE)
		std::cout << "step " << i << ": " << lastOutput << " -> " << nextInput << std::endl;
#endif
	}

	// Test for convergence on setPoint with 5% error margin. Note that this only tests
	// the 500th step, so there is a small chance it will give false positives for
	// solutions that have not converged (particularly for slow reaction setups).
	if (setPoint * 0.95 <= nextInput && nextInput <= setPoint * 1.05) {
		isSuccess = true;
	}
	else {
		isSuccess = false;
	}

	if (!shouldConverge) {
		// If we're expecting non-convergence, then non-convergence means the test is 
		// successful.
		isSuccess = !isSuccess;
	}

	cout << functionName << ": " << (isSuccess ? "PASSED" : "FAILED") << endl;
	return isSuccess;
}

// Runs initialization for the delay tests before testing for conversion
bool testDelayConvergence(double setPoint, double proportionWeight, double integralWeight,
	double derivativeWeight, double(*processFunction)(double), string functionName,
	bool shouldConverge = true, unsigned int stepsToDelay = 5) {

	// Empty the queue and initialize it to zeros
	while (!delayQueue.empty()) {
		delayQueue.pop();
	}
	for (unsigned int i = 0; i < stepsToDelay; ++i) {
		delayQueue.push(0);
	}

	return testConvergence(setPoint, proportionWeight, integralWeight, derivativeWeight,
		processFunction, functionName, shouldConverge);
}

int main(int argc, char* argv[]) {

	// To rerun a random test, replace this seed with the seed from the previous run. 
	unsigned int seed = (unsigned int)time(NULL);
	rng.seed(seed);
    cout << "Beginning tests with random seed " << seed << "... " << endl;

	// Perfect signal - no lag, offset, or interference
	testConvergence(1000, 0, 200, 0, scale, "idealTest");
	// Run the same test with different values of P and D just to make sure that they
	// do, in fact, converge. The PID values used for these tests are slightly arbitrary
	// values that are simply known to converge eventually.
	testConvergence(1000, 500, 200, 0, scale, "idealTest2");
	testConvergence(1000, 500, 200, 20, scale, "idealTest3");

	testConvergence(1000, 500, 200, 20, offset, "offsetTest");

	testDelayConvergence(1000, 5000, 200, 20, delay, "delayTest");
	testDelayConvergence(1000, 5000, 1300, 340, delay, "delayTest2");

	testConvergence(1000, 4000, 1300, 20, noise, "noiseTest");
    
    cout << "Tests complete!" << endl;

	// There's plenty more tests I could add here to test each of the parameters more
	// thoroughly as well as testing under more process profiles and with different
	// tuning of the setPoint. I'm also missing some negative cases that demonstrate
	// poorly-tuned PID values. This seems like a good demonstration though.

    return 0;
}