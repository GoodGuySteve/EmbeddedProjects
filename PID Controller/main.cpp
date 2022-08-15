/*
    Code for testing the PID Controller class.
*/
#include "PidController.h"
#include <iostream>
#include <sstream>

using namespace std;

/*------------------------------
  Process functions - for simulating different effects of the controller's output
  on the input signal
  ------------------------------*/

// Simply take the output and scale it down before it is fed back into the controller
double scale(double pidOutput) {
	return pidOutput / 10000.0;
}

/*------------------------------
  Test functions - for defining different tests to run on the PidController class.
  This also demonstrates how the class is used. 
  ------------------------------*/

bool testConvergence(double setPoint, double proportionWeight, double integralWeight,
	      double derivativeWeight, double (*processFunction)(double), string functionName,
          bool shouldConverge = true) {
	const int numSteps = 500;

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

int main(int argc, char* argv[]) {

    cout << "Beginning tests... " << endl;

	// Perfect signal - no lag, offset, or interference
	testConvergence(1000, 0, 200, 0, scale, "idealTest");
	// Run the same test with different values of P and D just to make sure that they
	// do, in fact, converge. 
	testConvergence(1000, 500, 200, 0, scale, "idealTest2");
	testConvergence(1000, 500, 200, 20, scale, "idealTest3");
    
    cout << "Tests complete!" << endl;

    return 0;
}