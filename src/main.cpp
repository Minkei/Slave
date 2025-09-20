#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
#include "../lib/F3ApplicationLayer/PurePersuit/purePursuit.h"
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

// Pure Pursuit control variables
bool purePursuitActive = false;
unsigned long lastPurePursuitUpdate = 0;
const unsigned long PURE_PURSUIT_UPDATE_INTERVAL = 50; // 50ms = 20Hz

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

// Pure Pursuit Controller with optimized parameters
PurePursuitController purePursuitController;

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

      // Stop Pure Pursuit if manual velocity command
      if (purePursuitActive)
      {
        purePursuitController.reset();
        purePursuitActive = false;
        Serial.println("Pure Pursuit disabled - manual control");
      }

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
    purePursuitController.reset();
    kiddoCar.setVelocity(0, 0);
    purePursuitActive = false;
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
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
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

void handlePurePursuitCommands(String command)
{
  if (command.startsWith("goto "))
  {
    int space1 = command.indexOf(' ', 5);
    int space2 = command.indexOf(' ', space1 + 1);

    if (space1 > 0)
    {
      float x = command.substring(5, space1).toFloat();
      float y = (space2 > 0) ? command.substring(space1 + 1, space2).toFloat() : command.substring(space1 + 1).toFloat();
      float theta = (space2 > 0) ? command.substring(space2 + 1).toFloat() * M_PI / 180.0f : 0.0f;

      // Set target with current pose as start
      Pose2D currentPose = odometry.getPose();
      TargetPoint target = {x, y, theta, true};
      purePursuitController.setTargetWithStartPose(target, currentPose);

      purePursuitActive = true;
      lastPurePursuitUpdate = 0; // Force immediate update

      Serial.print("Target set to: (");
      Serial.print(x, 3);
      Serial.print(", ");
      Serial.print(y, 3);
      Serial.print(", ");
      Serial.print(theta * 180.0f / M_PI, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("Format: goto <x> <y> [theta_degrees]");
    }
  }
  else if (command == "ppstop")
  {
    purePursuitController.reset();
    kiddoCar.setVelocity(0, 0);
    purePursuitActive = false;
    Serial.println("Pure Pursuit stopped");
  }
  else if (command == "ppstatus")
  {
    purePursuitController.printDetailedStatus();
  }
  else if (command == "ppemergency")
  {
    purePursuitController.emergencyStop();
    kiddoCar.setVelocity(0, 0);
    purePursuitActive = false;
    Serial.println("Pure Pursuit EMERGENCY STOP");
  }
  else if (command.startsWith("ppparams "))
  {
    int space1 = command.indexOf(' ', 9);
    int space2 = command.indexOf(' ', space1 + 1);

    if (space1 > 0 && space2 > 0)
    {
      PurePursuitParams params = purePursuitController.getParams();
      params.max_speed = command.substring(9, space1).toFloat();
      params.min_speed = command.substring(space1 + 1, space2).toFloat();
      params.lookahead_dist = command.substring(space2 + 1).toFloat();

      purePursuitController.setParams(params);
      Serial.print("PP Params - Max:");
      Serial.print(params.max_speed, 3);
      Serial.print(" Min:");
      Serial.print(params.min_speed, 3);
      Serial.print(" Lookahead:");
      Serial.println(params.lookahead_dist, 3);
    }
    else
    {
      Serial.println("Format: ppparams <max_speed> <min_speed> <lookahead>");
    }
  }
  else if (command.startsWith("ppfast "))
  {
    bool enable = (command.substring(7) == "on");
    purePursuitController.enableFastMath(enable);
    Serial.print("Fast math ");
    Serial.println(enable ? "ENABLED" : "DISABLED");
  }
  else if (command.startsWith("ppadaptive "))
  {
    bool enable = (command.substring(11) == "on");
    purePursuitController.enableAdaptiveLookahead(enable);
    Serial.print("Adaptive lookahead ");
    Serial.println(enable ? "ENABLED" : "DISABLED");
  }
  else if (command.startsWith("pptimeout "))
  {
    float timeout = command.substring(10).toFloat() * 1000; // Convert to ms
    purePursuitController.setTimeout(timeout);
    Serial.print("Timeout set to ");
    Serial.print(timeout / 1000.0f);
    Serial.println(" seconds");
  }
  else if (command.startsWith("ppspeed "))
  {
    float speed = command.substring(8).toFloat();
    purePursuitController.setMaxSpeed(speed);
    Serial.print("Max speed set to ");
    Serial.print(speed, 3);
    Serial.println(" m/s");
  }
  else if (command.startsWith("pplookahead "))
  {
    float lookahead = command.substring(12).toFloat();
    purePursuitController.setLookaheadDistance(lookahead);
    Serial.print("Lookahead distance set to ");
    Serial.print(lookahead, 3);
    Serial.println(" m");
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

  // Serial.println("\n--- Pure Pursuit ---");
  // Serial.println("goto <x> <y> [θ°] - Move to target position");
  // Serial.println("ppstop            - Stop Pure Pursuit");
  // Serial.println("ppemergency       - Emergency stop");
  // Serial.println("ppstatus          - Show detailed PP status");
  // Serial.println("ppparams <ms><mn><l> - Set max_speed, min_speed, lookahead");
  // Serial.println("ppfast on/off     - Enable/disable fast math");
  // Serial.println("ppadaptive on/off - Enable/disable adaptive lookahead");
  // Serial.println("pptimeout <sec>   - Set timeout in seconds");
  // Serial.println("ppspeed <m/s>     - Set max speed");
  // Serial.println("pplookahead <m>   - Set lookahead distance");

  Serial.println("\n--- Status & Debug ---");
  Serial.println("status            - Show system status");
  Serial.println("on/off            - Debug mode toggle");
  Serial.println("kintest           - Test kinematics");

  Serial.println("\n--- Quick Examples ---");
  Serial.println("vel 0.1 0         - Move forward 10cm/s");
  Serial.println("goto 0.2 0        - Move 20cm forward");
  Serial.println("goto 0.1 0.1 90   - Move to (10cm,10cm) facing 90°");
  Serial.println("ppparams 0.08 0.02 0.05 - Set PP parameters");
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

    Serial.print("Pure Pursuit: ");
    Serial.print(purePursuitActive ? "ACTIVE" : "INACTIVE");
    if (purePursuitActive)
    {
      Serial.print(" - ");
      Serial.print(purePursuitController.getStateString());
    }
    Serial.println();
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
  else if (command == "help")
  {
    showHelp();
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

    // Pure Pursuit commands
    // else if (command.startsWith("goto ") || command.startsWith("pp")) {
    //   handlePurePursuitCommands(command);
    // }

    // Status and debug commands
    else if (command == "status" || command == "on" || command == "off")
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

    // Setup Pure Pursuit with optimized parameters
    // PurePursuitParams ppParams = PurePursuitController::getDefaultParams();
    // ppParams.enable_fast_math = true;
    // ppParams.enable_adaptive_lookahead = false;
    // ppParams.timeout_ms = 30000; // 30 seconds
    // purePursuitController.setParams(ppParams);

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

      // Pure Pursuit update với tần số cố định
      // unsigned long currentTime = millis();
      // if (purePursuitActive && (currentTime - lastPurePursuitUpdate >= PURE_PURSUIT_UPDATE_INTERVAL)) {
      //   Pose2D currentPose = odometry.getPose();
      //   TargetPoint emptyTarget = {0, 0, 0, false}; // Không update target trong loop

      //   RobotVelocity ppVelocity = purePursuitController.update(currentPose, emptyTarget);

      //   // Handle Pure Pursuit states
      //   switch(purePursuitController.getState()) {
      //     case PP_FOLLOWING:
      //       kiddoCar.setVelocity(ppVelocity.linear, ppVelocity.angular);
      //       break;

      //     case PP_GOAL_REACHED:
      //       kiddoCar.setVelocity(0, 0);
      //       purePursuitActive = false;
      //       Serial.println("✓ Pure Pursuit: Goal reached!");
      //       break;

      //     case PP_TIMEOUT:
      //       kiddoCar.setVelocity(0, 0);
      //       purePursuitActive = false;
      //       Serial.println("⚠ Pure Pursuit: Timeout!");
      //       break;

      //     case PP_SAFETY_STOP:
      //       kiddoCar.setVelocity(0, 0);
      //       purePursuitActive = false;
      //       Serial.println("⚠ Pure Pursuit: Safety stop!");
      //       break;

      //     case PP_ERROR:
      //       kiddoCar.setVelocity(0, 0);
      //       purePursuitActive = false;
      //       Serial.println("✗ Pure Pursuit: Error occurred!");
      //       break;

      //     case PP_IDLE:
      //       // Do nothing
      //       break;
      //   }

      //   lastPurePursuitUpdate = currentTime;
      // }

      // Debug output
      if (debugMode) {
        // purePursuitController.printStatus(true, 10);  // Every 200ms when debug enabled
        wheelLeft.printStatus(true, 10);
      }


      dataReady = false;
    }
  }