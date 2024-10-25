#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

// An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  // Note the trailing 'f' in the number to force single precision floating point.

Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

// We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// Some constants that we may use:
const float mass = 30e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  // MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  // MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  // MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f;  // [s] period between successive calls to MainLoop

Vec3f estGyroBias = Vec3f(0, 0, 0);

// Estimators for roll, pitch, and yaw angles
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;

// Trade-off factor for the complementary filter
const float rho = 0.01f;

MainLoopOutput MainLoop(MainLoopInput const &in) {
    MainLoopOutput outVals;

    // Gyroscope bias estimation during the first second
    if (in.currentTime < 1.0f) {
        estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
    }
    Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

    // Compute accelerometer-based estimates of roll and pitch
    float g_mag = gravity;
    float phi_meas = in.imuMeasurement.accelerometer.y / g_mag; // roll measurement
    float theta_meas = -in.imuMeasurement.accelerometer.x / g_mag; // pitch measurement

    // Combined roll estimation (using accelerometer and gyroscope data)
    estRoll = (1 - rho) * (estRoll + dt * rateGyro_corr.x) + rho * phi_meas;

    // Combined pitch estimation (using accelerometer and gyroscope data)
    estPitch = (1 - rho) * (estPitch + dt * rateGyro_corr.y) + rho * theta_meas;

    // Update yaw using only gyroscope data (integration)
    estYaw = estYaw + dt * rateGyro_corr.z;

    // Send the estimated attitude to telemetry
    outVals.telemetryOutputs_plusMinus100[0] = estRoll;
    outVals.telemetryOutputs_plusMinus100[1] = estPitch;
    outVals.telemetryOutputs_plusMinus100[2] = estYaw;

    // Set motor commands (currently zero)
    outVals.motorCommand1 = 0;
    outVals.motorCommand2 = 0;
    outVals.motorCommand3 = 0;
    outVals.motorCommand4 = 0;

    // Copy the inputs and outputs for debugging
    lastMainLoopInputs = in;
    lastMainLoopOutputs = outVals;
    return outVals;
}

void PrintStatus() {
    // Accelerometer readings
    printf("Acc: x=%6.3f, y=%6.3f, z=%6.3f\n", 
           double(lastMainLoopInputs.imuMeasurement.accelerometer.x),
           double(lastMainLoopInputs.imuMeasurement.accelerometer.y),
           double(lastMainLoopInputs.imuMeasurement.accelerometer.z));

    // Raw gyroscope readings
    printf("Gyro (raw): x=%6.3f, y=%6.3f, z=%6.3f\n", 
           double(lastMainLoopInputs.imuMeasurement.rateGyro.x),
           double(lastMainLoopInputs.imuMeasurement.rateGyro.y),
           double(lastMainLoopInputs.imuMeasurement.rateGyro.z));

    // Corrected gyroscope readings
    Vec3f rateGyro_corr = lastMainLoopInputs.imuMeasurement.rateGyro - estGyroBias;
    printf("Gyro (corrected): x=%6.3f, y=%6.3f, z=%6.3f\n", 
           double(rateGyro_corr.x), double(rateGyro_corr.y), double(rateGyro_corr.z));

    // Gyroscope bias
    printf("Gyro Bias: x=%6.3f, y=%6.3f, z=%6.3f\n", 
           double(estGyroBias.x), double(estGyroBias.y), double(estGyroBias.z));

    // Estimated roll, pitch, and yaw
    printf("Estimated Attitude: Roll=%6.3f, Pitch=%6.3f, Yaw=%6.3f\n", 
           double(estRoll), double(estPitch), double(estYaw));

    // Example variables
    printf("Example variable values:\n");
    printf("  exampleVariable_int = %d\n", exampleVariable_int);
    printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));
    printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n", 
           double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y), double(exampleVariable_Vec3f.z));

    // Last main loop inputs
    printf("Last main loop inputs:\n");
    printf("  batt voltage = %6.3f\n", double(lastMainLoopInputs.batteryVoltage.value));
    printf("  JS buttons: ");
    if (lastMainLoopInputs.joystickInput.buttonRed) printf("buttonRed ");
    if (lastMainLoopInputs.joystickInput.buttonGreen) printf("buttonGreen ");
    if (lastMainLoopInputs.joystickInput.buttonBlue) printf("buttonBlue ");
    if (lastMainLoopInputs.joystickInput.buttonYellow) printf("buttonYellow ");
    if (lastMainLoopInputs.joystickInput.buttonStart) printf("buttonStart ");
    if (lastMainLoopInputs.joystickInput.buttonSelect) printf("buttonSelect ");
    printf("\n");

    // Last main loop outputs
    printf("Last main loop outputs:\n");
    printf("  motor command 1 = %6.3f\n", double(lastMainLoopOutputs.motorCommand1));
}