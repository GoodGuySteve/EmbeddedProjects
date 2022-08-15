#include "PidController.h"

PidController::PidController() {
    this->setPoint = 0;
    this->proportionWeight = 0;
    this->integralWeight = 0;
    this->derivativeWeight = 0;
    this->cumulativeIntegral = 0;
    this->prevError = 0;
}

PidController::PidController(double proportionWeight, double integralWeight, double derivativeWeight, double setPoint) {
    *this = PidController();
    this->setPid(proportionWeight, integralWeight, derivativeWeight);
    this->setSetPoint(setPoint);
}

PidController& PidController::operator=(const PidController& other) { 
    this->proportionWeight = other.proportionWeight;
    this->integralWeight = other.integralWeight;
    this->derivativeWeight = other.derivativeWeight;
    this->setPoint = other.setPoint;
    this->prevError = other.prevError;
    this->cumulativeIntegral = other.cumulativeIntegral;

    return *this;
}

PidController::~PidController() {}

/* Given the next step of the input signal, compute the output signal to approach the
   set point by combining all three PID components. This assumes that the input is
   given at a constant rate. */
double PidController::nextOutput(double input) {

    double error = this->setPoint - input;
    // Proportion simply compares error to the set point
    double proportionOutput = error * this->proportionWeight; 

    // Integral is similar to proportion, but accumulates over many steps
    this->cumulativeIntegral += error * this->integralWeight;
    double integralOutput = this->cumulativeIntegral;

    // Derivative simply compares error values and offsets accordingly. This should be
    // in the reverse direction of the other terms.
    double derivativeOutput = this->derivativeWeight * (error - this->prevError);
    this->prevError = error;

    return proportionOutput + integralOutput + derivativeOutput;
}

double PidController::getSetPoint(void) {
    return this->setPoint;
}

/* Change the target value of the input signal that the PID controller is aiming to
   reach and maintain */ 
void PidController::setSetPoint(double setPoint) {
    this->setPoint = setPoint;
}

double PidController::getProportionWeight(void) {
    return this->proportionWeight;
}

double PidController::getIntegralWeight(void) {
    return this->integralWeight;
}

double PidController::getDerivativeWeight(void) {
    return this->derivativeWeight;
}

/* Sets all three of the PID tuning values at once to the specified values */
void PidController::setPid(double proportionWeight, double integralWeight, double derivativeWeight) {
    this->proportionWeight = proportionWeight;
    this->integralWeight = integralWeight;
    this->derivativeWeight = derivativeWeight;
}