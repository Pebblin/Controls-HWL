#include "UserCode.hpp"
//#include "UtilityFunctions.hpp"
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
const float mass = 40e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  // MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  // MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  // MMOI about z axis [kg.m^2]
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

const float dt = 1.0f / 500.0f;  // [s] period between successive calls to MainLoop

Vec3f estGyroBias = Vec3f(0, 0, 0);

// Estimators for roll, pitch, and yaw angles
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;

// Trade-off factor for the complementary filter
const float rho = 0.01f;

float l = 0.033; // propeller distance in meters
float kappa = 0.01; // coupling coefficient

// Constants for the angle control time constants
const float timeConstant_rollAngle = 0.25f;  // Roll angle time constant [s]
const float timeConstant_pitchAngle = timeConstant_rollAngle;  // Pitch angle time constant [s]
const float timeConstant_yawAngle = 0.25f;  // Yaw angle time constant [s]

// Constants for the controller time constants
const float timeConstant_rollRate = 0.04f;  // Roll rate time constant [s]
const float timeConstant_pitchRate = timeConstant_rollRate;  // Pitch rate time constant [s]
const float timeConstant_yawRate = 0.05f;    // Yaw rate time constant [s]

// Constant for horizontal velocity
const float timeConst_horizVel = 2.0f;  // Horizontal velocity time constant [s]



// Desired angles (set to zero as per instructions)
Vec3f desAngle = Vec3f(0.0f, 0.0f, 0.0f);

// Desired normalized thrust (acceleration) in m/s^2
float desNormalizedAcceleration = 10.0f;

// Desired angular velocity in rad/s
Vec3f desAngularVel = Vec3f(0.0f, 0.0f, 0.0f);

//initialize motorforces
float f1,f2,f3,f4;


//Estimate Variables
float estHeight = 0.0f;
float estVelocity_1 = 0.0f;
float estVelocity_2 = 0.0f;
float estVelocity_3 = 0.0f;

//Last Variables
float lastHeightMeas_meas = 0.0f;
float lastHeightMeas_time = 0.0f;



float pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // Replace these two coefficients with experimentally determined values
  float a = -83.30318f;  // zeroth-order term
  float b = 0.12345f;    // first-order term

  // Safety check to return 0 for negative or invalid speeds
  if (desiredSpeed_rad_per_sec <= 0) {
    return 0;
  }

  return float(a + b * desiredSpeed_rad_per_sec);

}

float speedFromForce(float desiredForce_N) {
  // replace this with your determined constant:
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 2.1e-08f;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}


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
    estRoll = (1.0f - rho) * (estRoll + dt * rateGyro_corr.x) + rho * phi_meas;

    // Combined pitch estimation (using accelerometer and gyroscope data)
    estPitch = (1.0f - rho) * (estPitch + dt * rateGyro_corr.y) + rho * theta_meas;

    // Update yaw using only gyroscope data (integration)
    estYaw = estYaw + dt * rateGyro_corr.z;

    // Send the estimated attitude to telemetry
    outVals.telemetryOutputs_plusMinus100[0] = estRoll;
    outVals.telemetryOutputs_plusMinus100[1] = estPitch;
    outVals.telemetryOutputs_plusMinus100[2] = estYaw;


    // In MainLoop():
    // Height estimator - prediction step:
    estHeight = estHeight + estVelocity_3 * dt;
    estVelocity_3 = estVelocity_3 + 0 * dt; // Assume constant acceleration

    // Correction step, directly after the prediction step:
    const float mixHeight = 0.3f;

    if (in.heightSensor.updated) {
        // Check that the measurement is reasonable
        if (in.heightSensor.value < 5.0f) {
            float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
            estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;

            float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);
            estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;

            // Store this measurement for the next velocity update
            lastHeightMeas_meas = hMeas;
            lastHeightMeas_time = in.currentTime;
        }
    }

    //Horizontal State Estimate
    // Prediction Step
    // Assume velocity is constant:
    estVelocity_1 = estVelocity_1 + 0 * dt;
    estVelocity_2 = estVelocity_2 + 0 * dt;

    // Correction Step
    const float mixHorizVel = 0.1f;

    if (in.opticalFlowSensor.updated) {
      float sigma_1 = in.opticalFlowSensor.value_x;
      float sigma_2 = in.opticalFlowSensor.value_y;
      float div = cosf(estRoll) * cosf(estPitch);

      if (div > 0.5f) {
        float deltaPredict = estHeight / div; // Delta in the equation
        float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
        float v2Meas = (-sigma_2 + in.imuMeasurement.rateGyro.x) * deltaPredict;

        estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
        estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
      }
    }

    // Compute desired acceleration components
    float desAcc1 = -(1 / timeConst_horizVel) * estVelocity_1;
    float desAcc2 = -(1 / timeConst_horizVel) * estVelocity_2;

    // Desired angle computation
    desAngle.x = -desAcc2 / gravity; // Desired Roll
    desAngle.y = desAcc1 / gravity; // Desired Pitch
    desAngle.z = 0; // Desired Yaw

    // Setting desired height for total thrust command
    const float desHeight = 0.5f;
    const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3
        - natFreq_height * natFreq_height * (estHeight - desHeight);

    desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch)); // Modify desNormalizedAcceleration


    //  outVals.telemetryOutputs_plusMinus100[9] = desAngle.y;

    // Commanded angular velocities for the angle controller
       Vec3f cmdAngVel;
       cmdAngVel.x = -(1.0f / timeConstant_rollAngle) * (estRoll - desAngle.x);
       cmdAngVel.y = -(1.0f / timeConstant_pitchAngle) * (estPitch - desAngle.y);
       cmdAngVel.z = -(1.0f / timeConstant_yawAngle) * (estYaw - desAngle.z);

//       outVals.telemetryOutputs_plusMinus100[6] = cmdAngVel.x;
//       outVals.telemetryOutputs_plusMinus100[7] = cmdAngVel.y;
//       outVals.telemetryOutputs_plusMinus100[8] = cmdAngVel.z;




    // Commanded angular accelerations calculation based on the rate error

    Vec3f cmdAngAcc;
    cmdAngAcc.x = -(1.0f / timeConstant_rollRate) * (rateGyro_corr.x - cmdAngVel.x);
    cmdAngAcc.y = -(1.0f / timeConstant_pitchRate) * (rateGyro_corr.y - cmdAngVel.y);
    cmdAngAcc.z = -(1.0f / timeConstant_yawRate) * (rateGyro_corr.z - cmdAngVel.z);

    // Send the commanded angular acceleration to telemetry for monitoring
//    outVals.telemetryOutputs_plusMinus100[3] = cmdAngAcc.x;
//    outVals.telemetryOutputs_plusMinus100[4] = cmdAngAcc.y;
//    outVals.telemetryOutputs_plusMinus100[5] = cmdAngAcc.z;

    // Calculate the total desired force
    float desiredForce = mass * desNormalizedAcceleration;

    //

    Vec3f desiredTorque = Vec3f(cmdAngAcc.x * inertia_xx, cmdAngAcc.y * inertia_yy, cmdAngAcc.z * inertia_zz);



    // Apply the mixer matrix to convert to motor forces
    f1 = 0.25f * (desiredForce + desiredTorque.x / l - desiredTorque.y / l + desiredTorque.z / kappa);
    f2 = 0.25f * (desiredForce - desiredTorque.x / l - desiredTorque.y / l - desiredTorque.z / kappa);
    f3 = 0.25f * (desiredForce - desiredTorque.x / l + desiredTorque.y / l + desiredTorque.z / kappa);
    f4 = 0.25f * (desiredForce + desiredTorque.x / l + desiredTorque.y / l - desiredTorque.z / kappa);





         outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(f1));
         outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(f2));
         outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(f3));
         outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(f4));




//    // Log State Estimates
    outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
    outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
    outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
    outVals.telemetryOutputs_plusMinus100[6] = estHeight;
    outVals.telemetryOutputs_plusMinus100[7] = desAngle.x;
    outVals.telemetryOutputs_plusMinus100[8] = desAngle.y;
    outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
    outVals.telemetryOutputs_plusMinus100[10] = f1 + f2 + f3 + f4;
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

    //print motor forces
    printf("Motor Forces: f1=%6.3f, f2=%6.3f, f3=%6.3f, f4=%6.3f\n", f1, f2, f3, f4);
    //print motor speeds
    printf("Motor Speeds: f1=%6.3f, f2=%6.3f, f3=%6.3f, f4=%6.3f\n", speedFromForce(f1), speedFromForce(f2), speedFromForce(f3), speedFromForce(f4));
    //print motor COMMANDS
    printf("Motor pwm: f1=%6.3f, f2=%6.3f, f3=%6.3f, f4=%6.3f\n", pwmCommandFromSpeed(speedFromForce(f1)), pwmCommandFromSpeed(speedFromForce(f2)), pwmCommandFromSpeed(speedFromForce(f3)), pwmCommandFromSpeed(speedFromForce(f4)));


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
    printf("  motor command 2 = %6.3f\n", double(lastMainLoopOutputs.motorCommand2));
    printf("  motor command 3 = %6.3f\n", double(lastMainLoopOutputs.motorCommand3));
    printf("  motor command 4 = %6.3f\n", double(lastMainLoopOutputs.motorCommand4));

    printf("Last range = %6.3fm, ", static_cast<double>(lastMainLoopInputs.heightSensor.value));
    printf("Last flow: x=%6.3f, y=%6.3f\n",
    static_cast<double>(lastMainLoopInputs.opticalFlowSensor.value_x),
    static_cast<double>(lastMainLoopInputs.opticalFlowSensor.value_y));
}
