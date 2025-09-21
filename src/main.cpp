#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
#include <hardware/timer.h>

// Wheel objects (RPM controller, lowest controller level)
Wheel wheelLeft(
    PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, PWM_FREQ,
    PIN_MOTOR_L_ENCODER_A, PIN_MOTOR_L_ENCODER_B, REDUCER_RATIO, ENCODER_RESOLUTION, X4_MODE,
    KP, KI, KD, SAMPLE_TIME_MS);

Wheel wheelRight(
    PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_FREQ,
    PIN_MOTOR_R_ENCODER_A, PIN_MOTOR_R_ENCODER_B, REDUCER_RATIO, ENCODER_RESOLUTION, X4_MODE,
    KP, KI, KD, SAMPLE_TIME_MS);

// Control variables
volatile bool dataReady = false;
volatile bool commandReady = false;
volatile bool debugMode = false;
String inputString = "";

// PID tuning variables
float currentKP = KP;
float currentKI = KI;
float currentKD = KD;

// Kinematics
Kinematics kinematics(WHEEL_RADIUS_M, WHEEL_BASE_M);

// Differential Drive
DifferentialDrive kiddoCar(&wheelLeft, &wheelRight, WHEEL_RADIUS_M, WHEEL_BASE_M, MAX_RPM);

// Odometry
Odometry odometry(&kinematics, ENCODER_RESOLUTION, REDUCER_RATIO, X4_MODE);

bool sysUpdateCallback(struct repeating_timer *t)
{
  if (!dataReady)
  {
    wheelLeft.update();
    wheelRight.update();
    dataReady = true;
  }
  return true;
}

void handleMotionCommands(String command)
{
  if (command.startsWith("vel "))
  {
    int spaceIndex = command.indexOf(' ', 4);
    if (spaceIndex > 0)
    {
      float linear = command.substring(4, spaceIndex).toFloat();
      float angular = command.substring(spaceIndex + 1).toFloat();

      kiddoCar.setVelocity(linear, angular);
      Serial.print("Velocity set to ");
      Serial.print(linear, 3);
      Serial.print(" m/s linear, ");
      Serial.print(angular, 3);
      Serial.println(" rad/s angular");
    }
    else
    {
      Serial.println("Format: vel <linear> <angular>");
    }
  }
  else if (command == "stop")
  {
    kiddoCar.setVelocity(0, 0);
    Serial.println("All motion stopped");
  }
}

void handlePIDCommands(String command)
{
  if (command.startsWith("kp "))
  {
    currentKP = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KP set to: ");
    Serial.println(currentKP, 4);
  }
  else if (command.startsWith("ki "))
  {
    currentKI = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KI set to: ");
    Serial.println(currentKI, 4);
  }
  else if (command.startsWith("kd "))
  {
    currentKD = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KD set to: ");
    Serial.println(currentKD, 4);
  }
  else if (command.startsWith("pid "))
  {
    int firstSpace = command.indexOf(' ', 4);
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0)
    {
      currentKP = command.substring(4, firstSpace).toFloat();
      currentKI = command.substring(firstSpace + 1, secondSpace).toFloat();
      currentKD = command.substring(secondSpace + 1).toFloat();
      wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
      wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
      Serial.print("PID set - KP:");
      Serial.print(currentKP, 4);
      Serial.print(" KI:");
      Serial.print(currentKI, 4);
      Serial.print(" KD:");
      Serial.println(currentKD, 4);
    }
    else
    {
      Serial.println("Format: pid <kp> <ki> <kd>");
    }
  }
  else if (command == "pidreset")
  {
    wheelLeft.resetPID();
    wheelRight.resetPID();
    Serial.println("PID reset");
  }
}

void handleAccelerationCommands(String command)
{
  if (command.startsWith("accel "))
  {
    float accel = command.substring(6).toFloat();
    if (accel > 0 && accel <= 200)
    {
      wheelLeft.setMaxAcceleration(accel);
      wheelRight.setMaxAcceleration(accel);
      Serial.print("Max acceleration set to ");
      Serial.print(accel);
      Serial.println(" RPM/s");
    }
    else
    {
      Serial.println("Invalid acceleration. Range: 1-200 RPM/s");
    }
  }
  else if (command == "accelon")
  {
    wheelLeft.enableAccelerationLimiting(true);
    wheelRight.enableAccelerationLimiting(true);
    Serial.println("Acceleration limiting ENABLED");
  }
  else if (command == "acceloff")
  {
    wheelLeft.enableAccelerationLimiting(false);
    wheelRight.enableAccelerationLimiting(false);
    Serial.println("Acceleration limiting DISABLED");
  }
  else if (command == "accelstatus")
  {
    Serial.println("=== ACCELERATION STATUS ===");
    Serial.print("Left wheel - Max accel: ");
    Serial.print(wheelLeft.getMaxAcceleration());
    Serial.print(" RPM/s, Enabled: ");
    Serial.print(wheelLeft.isAccelerationEnabled() ? "YES" : "NO");
    Serial.print(", Accelerating: ");
    Serial.println(wheelLeft.isAccelerating() ? "YES" : "NO");

    Serial.print("Right wheel - Max accel: ");
    Serial.print(wheelRight.getMaxAcceleration());
    Serial.print(" RPM/s, Enabled: ");
    Serial.print(wheelRight.isAccelerationEnabled() ? "YES" : "NO");
    Serial.print(", Accelerating: ");
    Serial.println(wheelRight.isAccelerating() ? "YES" : "NO");
  }
}

void handleOdometryCommands(String command)
{
  if (command == "pose")
  {
    Pose2D pose = odometry.getPose();
    Serial.print("Position: X=");
    Serial.print(pose.x, 3);
    Serial.print("m, Y=");
    Serial.print(pose.y, 3);
    Serial.print("m, Theta=");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println("°");
  }
  else if (command == "resetpose")
  {
    odometry.resetPose();
    Serial.println("Pose reset to (0,0,0°)");
  }
  else if (command.startsWith("setpose "))
  {
    int space1 = command.indexOf(' ', 8);
    int space2 = command.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0)
    {
      float x = command.substring(8, space1).toFloat();
      float y = command.substring(space1 + 1, space2).toFloat();
      float theta_deg = command.substring(space2 + 1).toFloat();
      float theta_rad = theta_deg * M_PI / 180.0f;

      Pose2D newPose = {x, y, theta_rad};
      odometry.setPose(newPose);
      Serial.print("Pose set to: (");
      Serial.print(x, 2);
      Serial.print(", ");
      Serial.print(y, 2);
      Serial.print(", ");
      Serial.print(theta_deg, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("Format: setpose <x> <y> <theta_degrees>");
    }
  }
}

void showHelp() {
  Serial.println("\n=== ROBOT CONTROL SYSTEM ===");

  Serial.println("\n--- Motion Control ---");
  Serial.println("vel <lin> <ang>   - Set linear/angular velocity");
  Serial.println("stop              - Stop all motion");

  Serial.println("\n--- PID Control ---");
  Serial.println("kp/ki/kd <value>  - Set PID gains individually");
  Serial.println("pid <p> <i> <d>   - Set all PID values");
  Serial.println("pidreset          - Reset PID controllers");

  Serial.println("\n--- Acceleration ---");
  Serial.println("accel <rpm/s>     - Set max acceleration");
  Serial.println("accelon/acceloff  - Enable/disable acceleration limiting");
  Serial.println("accelstatus       - Show acceleration status");

  Serial.println("\n--- Odometry ---");
  Serial.println("pose              - Show current position");
  Serial.println("resetpose         - Reset position to origin");
  Serial.println("setpose <x> <y> <θ°> - Set position manually");

  Serial.println("\n--- Status & Debug ---");
  Serial.println("status            - Show system status");
  Serial.println("on/off            - Debug mode toggle");
  Serial.println("kintest           - Test kinematics");

  Serial.println("\n--- Quick Examples ---");
  Serial.println("vel 0.1 0         - Move forward 10cm/s");
  Serial.println("vel 0 0.5         - Rotate 0.5 rad/s");
  Serial.println("vel 0.1 0.3       - Move + rotate");
  Serial.println("pid 2.0 0.1 0.05  - Set PID gains");
  Serial.println("accel 30          - Set gentle acceleration");
}

void handleStatusCommands(String command) {
  if (command == "status")
  {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.print("Left RPM: ");
    Serial.print(wheelLeft.getCurrentRPM(), 2);
    Serial.print(", Right RPM: ");
    Serial.println(wheelRight.getCurrentRPM(), 2);

    Serial.print("PID - KP:");
    Serial.print(currentKP, 4);
    Serial.print(" KI:");
    Serial.print(currentKI, 4);
    Serial.print(" KD:");
    Serial.println(currentKD, 4);

    RobotVelocity currentVel = kiddoCar.getCurrentVelocity();
    Serial.print("Velocity - Linear: ");
    Serial.print(currentVel.linear, 3);
    Serial.print(" m/s, Angular: ");
    Serial.print(currentVel.angular, 3);
    Serial.println(" rad/s");

    Pose2D pose = odometry.getPose();
    Serial.print("Position: (");
    Serial.print(pose.x, 3);
    Serial.print(", ");
    Serial.print(pose.y, 3);
    Serial.print(", ");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println("°)");
  }
  else if (command == "on")
  {
    debugMode = true;
    Serial.println("Debug mode enabled");
  }
  else if (command == "off")
  {
    debugMode = false;
    Serial.println("Debug mode disabled");
  }
  else if (command == "kintest")
  {
    Serial.println("=== KINEMATICS TEST ===");
    WheelRPM rpm = {30.0f, 30.0f};
    RobotVelocity vel = kinematics.forwardKinematics(rpm);
    Serial.print("30 RPM both wheels = ");
    Serial.print(vel.linear, 4);
    Serial.print(" m/s linear, ");
    Serial.print(vel.angular, 4);
    Serial.println(" rad/s angular");
    
    // Test reverse
    RobotVelocity testVel = {0.1f, 0.0f}; // 10cm/s forward
    WheelRPM resultRPM = kinematics.inverseKinematics(testVel);
    Serial.print("0.1 m/s forward = Left:");
    Serial.print(resultRPM.left, 2);
    Serial.print(" RPM, Right:");
    Serial.print(resultRPM.right, 2);
    Serial.println(" RPM");
  }
}

void processCommand(String command)
{
  command.trim();
  command.toLowerCase();

  // Motion commands
  if (command.startsWith("vel ") || command == "stop")
  {
    handleMotionCommands(command);
  }
  // PID commands
  else if (command.startsWith("kp ") || command.startsWith("ki ") || command.startsWith("kd ") || command.startsWith("pid ") || command == "pidreset")
  {
    handlePIDCommands(command);
  }
  // Acceleration commands
  else if (command.startsWith("accel") || command == "accelon" || command == "acceloff")
  {
    handleAccelerationCommands(command);
  }
  // Odometry commands
  else if (command == "pose" || command == "resetpose" || command.startsWith("setpose "))
  {
    handleOdometryCommands(command);
  }
  // Status and debug commands
  else if (command == "status" || command == "on" || command == "off" || command == "kintest")
  {
    handleStatusCommands(command);
  }
  // Help
  else if (command == "help")
  {
    showHelp();
  }
  // Unknown command
  else if (command.length() > 0)
  {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\n=== ROBOT CONTROL SYSTEM ===");

  // Initialize wheels
  if (!wheelLeft.begin())
  {
    Serial.println("Failed to initialize left wheel.");
    return;
  }
  if (!wheelRight.begin())
  {
    Serial.println("Failed to initialize right wheel.");
    return;
  }
  Serial.println("Wheels initialized successfully.");

  // Initial wheel setup
  wheelLeft.setRawMotorSpeed(0.0);
  wheelRight.setRawMotorSpeed(0.0);

  // Setup filters
  wheelLeft.setRPMFilter(FilterFactory::createKalman(0.03f, 4.0f));
  wheelRight.setRPMFilter(FilterFactory::createKalman(0.03f, 4.0f));

  // Setup PID
  wheelLeft.setPIDTunings(KP, KI, KD);
  wheelRight.setPIDTunings(KP, KI, KD);
  wheelLeft.enablePID(true);
  wheelRight.enablePID(true);
  wheelLeft.setPIDSampleTime(SAMPLE_TIME_MS);
  wheelRight.setPIDSampleTime(SAMPLE_TIME_MS);
  wheelLeft.setPIDLimits(-100.0, 100.0);
  wheelRight.setPIDLimits(-100.0, 100.0);

  // Setup direction and acceleration
  wheelLeft.setDirection(false);
  wheelRight.setDirection(false);
  wheelLeft.setMaxAcceleration(40.0f);
  wheelRight.setMaxAcceleration(40.0f);
  wheelLeft.enableAccelerationLimiting(true);
  wheelRight.enableAccelerationLimiting(true);

  // Setup timer
  static struct repeating_timer timer;
  if (!add_repeating_timer_ms(SAMPLE_TIME_MS, sysUpdateCallback, NULL, &timer))
  {
    Serial.println("Failed to add timer");
    return;
  }

  Serial.println("System ready! Type 'help' for commands");
  Serial.print("PID: KP=");
  Serial.print(KP, 3);
  Serial.print(", KI=");
  Serial.print(KI, 3);
  Serial.print(", KD=");
  Serial.println(KD, 3);
  Serial.println();
}

void loop()
{
  // Handle serial commands
  while (Serial.available() > 0)
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      commandReady = true;
    }
  }

  if (commandReady)
  {
    processCommand(inputString);
    inputString = "";
    commandReady = false;
  }

  // Update system components
  if (dataReady)
  {
    long encoderLeft = wheelLeft.getPulsePosition();
    long encoderRight = wheelRight.getPulsePosition();
    odometry.update(encoderLeft, encoderRight);
    kiddoCar.update();

    // Debug output
    if (debugMode) {
      wheelLeft.printStatus(true, 10);
    }

    dataReady = false;
  }
}