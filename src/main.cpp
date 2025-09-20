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
  KP, KI, KD, SAMPLE_TIME_MS
);

Wheel wheelRight(
  PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_FREQ,
  PIN_MOTOR_R_ENCODER_A, PIN_MOTOR_R_ENCODER_B, REDUCER_RATIO, ENCODER_RESOLUTION, X4_MODE,
  KP, KI, KD, SAMPLE_TIME_MS
);

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

// Pure Pursuit Controller
PurePursuitController purePursuitController;

// Timer callback
bool sysUpdateCallback(struct repeating_timer *t) {
  if (!dataReady) {
    wheelLeft.update();
    wheelRight.update();
    // kiddoCar.update();
    // motionController.update();
    dataReady = true;
  }
  return true; 
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  // === VELOCITY CONTROL ===
  if (command.startsWith("vel ")) {
    int spaceIndex = command.indexOf(' ', 4);
    if (spaceIndex > 0) {
      float linear = command.substring(4, spaceIndex).toFloat();
      float angular = command.substring(spaceIndex + 1).toFloat();
      kiddoCar.setVelocity(linear, angular);
      Serial.print("Velocity set to ");
      Serial.print(linear, 3);
      Serial.print(" m/s linear, ");
      Serial.print(angular, 3);
      Serial.println(" rad/s angular");
    } else {
      Serial.println("Format: vel <linear> <angular>");
    }
  } else if (command == "stop") {
    kiddoCar.setVelocity(0, 0);
    Serial.println("Motion stopped");
    
  // === PID TUNING COMMANDS ===
  // } else if (command.startsWith("kp ")) {
  //   currentKP = command.substring(3).toFloat();
  //   wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
  //   wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
  //   Serial.print("KP set to: ");
  //   Serial.println(currentKP, 4); 
  // } else if (command.startsWith("ki ")) {
  //   currentKI = command.substring(3).toFloat();
  //   wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
  //   wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
  //   Serial.print("KI set to: ");
  //   Serial.println(currentKI, 4);
  // } else if (command.startsWith("kd ")) {
  //   currentKD = command.substring(3).toFloat();
  //   wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
  //   wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
  //   Serial.print("KD set to: ");
  //   Serial.println(currentKD, 4);
  // } else if (command.startsWith("pid ")) {
  //   // Format: "pid kp ki kd"
  //   int firstSpace = command.indexOf(' ', 4);
  //   int secondSpace = command.indexOf(' ', firstSpace + 1);
  //   if (firstSpace > 0 && secondSpace > 0) {
  //     currentKP = command.substring(4, firstSpace).toFloat();
  //     currentKI = command.substring(firstSpace + 1, secondSpace).toFloat();
  //     currentKD = command.substring(secondSpace + 1).toFloat();
  //     wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
  //     wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
  //     Serial.print("PID set - KP:");
  //     Serial.print(currentKP, 4);
  //     Serial.print(" KI:");
  //     Serial.print(currentKI, 4);
  //     Serial.print(" KD:");
  //     Serial.println(currentKD, 4);
  //   } else {
  //     Serial.println("Format: pid <kp> <ki> <kd>");
  //   }
  // } else if (command == "reset") {
  //   wheelLeft.resetPID();
  //   wheelRight.resetPID();
  //   kiddoCar.setVelocity(0, 0);
  //   Serial.println("PID reset and motion stopped");
    
  // === STATUS COMMANDS ===
  
} else if (command == "status") {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.print("Left - Current: ");
    Serial.print(wheelLeft.getCurrentRPM(), 2);
    Serial.println(" RPM");
    
    Serial.print("Right - Current: ");
    Serial.print(wheelRight.getCurrentRPM(), 2);
    Serial.println(" RPM");
    
    Serial.print("PID - KP:");
    Serial.print(currentKP, 4);
    Serial.print(" KI:");
    Serial.print(currentKI, 4);
    Serial.print(" KD:");
    Serial.println(currentKD, 4);
    
    // Current velocity
    RobotVelocity currentVel = kiddoCar.getCurrentVelocity();
    Serial.print("Velocity - Linear: ");
    Serial.print(currentVel.linear, 3);
    Serial.print(" m/s, Angular: ");
    Serial.print(currentVel.angular, 3);
    Serial.println(" rad/s");
    
    if (wheelLeft.getPulsePosition() || wheelRight.getPulsePosition()) {
      Serial.print("Encoders - Left: ");
      Serial.print(wheelLeft.getPulsePosition());
      Serial.print(", Right: ");
      Serial.println(wheelRight.getPulsePosition());
    }
  // === ACCELERATION TUNING COMMANDS ===
  } else if (command.startsWith("accel ")) {
    float accel = command.substring(6).toFloat();
    if (accel > 0 && accel <= 200) {
      wheelLeft.setMaxAcceleration(accel);
      wheelRight.setMaxAcceleration(accel);
      Serial.print("Max acceleration set to ");
      Serial.print(accel);
      Serial.println(" RPM/s");
    } else {
      Serial.println("Invalid acceleration. Range: 1-200 RPM/s");
    }
    
  } else if (command == "accelon") {
    wheelLeft.enableAccelerationLimiting(true);
    wheelRight.enableAccelerationLimiting(true);
    Serial.println("Acceleration limiting ENABLED");
    
  } else if (command == "acceloff") {
    wheelLeft.enableAccelerationLimiting(false);
    wheelRight.enableAccelerationLimiting(false);
    Serial.println("Acceleration limiting DISABLED");
    
  } else if (command == "accelstatus") {
    Serial.println("=== ACCELERATION STATUS ===");
    Serial.print("Left wheel - Max accel: ");
    Serial.print(wheelLeft.getMaxAcceleration());
    Serial.print(" RPM/s, Enabled: ");
    Serial.println(wheelLeft.isAccelerationEnabled() ? "YES" : "NO");
    
    Serial.print("Current target: ");
    Serial.print(wheelLeft.getCurrentTargetRPM());
    Serial.print(" RPM, Final target: ");
    Serial.print(wheelLeft.getFinalTargetRPM());
    Serial.print(" RPM, Accelerating: ");
    Serial.println(wheelLeft.isAccelerating() ? "YES" : "NO");
    
    // Same for right wheel
    Serial.print("Right wheel - Max accel: ");
    Serial.print(wheelRight.getMaxAcceleration());
    Serial.print(" RPM/s, Enabled: ");
    Serial.println(wheelRight.isAccelerationEnabled() ? "YES" : "NO");
    
    Serial.print("Current target: ");
    Serial.print(wheelRight.getCurrentTargetRPM());
    Serial.print(" RPM, Final target: ");
    Serial.print(wheelRight.getFinalTargetRPM());
    Serial.print(" RPM, Accelerating: ");
    Serial.println(wheelRight.isAccelerating() ? "YES" : "NO");
      
// === ODOMETRY COMMANDS ===
  } else if (command == "pose") {
    Pose2D pose = odometry.getPose();
    Serial.print("Position: X=");
    Serial.print(pose.x, 3);
    Serial.print("m, Y=");
    Serial.print(pose.y, 3);
    Serial.print("m, Theta=");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println(" deg");
      
  } else if (command == "resetpose") {
    odometry.resetPose();
    Serial.println("Pose reset to (0,0,0 deg)");
      
  } else if (command.startsWith("setpose ")) {
    // Format: "setpose x y theta_deg"
    int space1 = command.indexOf(' ', 8);
    int space2 = command.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
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
      Serial.println(" deg)");
    } else {
      Serial.println("Format: setpose <x> <y> <theta_degrees>");
    }
  // === PURE PURSUIT COMMANDS ===
  } else if (command.startsWith("goto ")) {
  // Format: "goto x y [theta]"
  int space1 = command.indexOf(' ', 5);
  int space2 = command.indexOf(' ', space1 + 1);
  
  if (space1 > 0) {
    float x = command.substring(5, space1).toFloat();
    float y = (space2 > 0) ? command.substring(space1 + 1, space2).toFloat() : 
                             command.substring(space1 + 1).toFloat();
    float theta = (space2 > 0) ? command.substring(space2 + 1).toFloat() : 0.0f;
    
    // Convert degrees to radians if needed
    theta = theta * M_PI / 180.0f;
    
    purePursuitController.setTarget(x, y, theta);
    Serial.print("Target set to: (");
    Serial.print(x, 3);
    Serial.print(", ");
    Serial.print(y, 3);
    Serial.print(", ");
    Serial.print(theta * 180.0f / M_PI, 1);
    Serial.println("°)");
  } else {
    Serial.println("Format: goto <x> <y> [theta_degrees]");
  }

  } else if (command == "ppstop") {
  purePursuitController.reset();
  kiddoCar.setVelocity(0, 0);
  Serial.println("Pure Pursuit stopped");

  } else if (command == "ppstatus") {
  Serial.println("=== PURE PURSUIT STATUS ===");
  TargetPoint target = purePursuitController.getCurrentTarget();
  Serial.print("Target: (");
  Serial.print(target.x, 3);
  Serial.print(", ");
  Serial.print(target.y, 3);
  Serial.print(", ");
  Serial.print(target.theta * 180.0f / M_PI, 1);
  Serial.print("°) Valid: ");
  Serial.println(target.valid ? "YES" : "NO");
  
  Serial.print("State: ");
  switch(purePursuitController.getState()) {
    case PP_IDLE: Serial.println("IDLE"); break;
    case PP_FOLLOWING: Serial.println("FOLLOWING"); break;
    case PP_GOAL_REACHED: Serial.println("GOAL REACHED"); break;
    case PP_ERROR: Serial.println("ERROR"); break;
  }
  
  Serial.print("Distance to target: ");
  Serial.print(purePursuitController.getDistanceToTarget(), 3);
  Serial.println(" m");
  
  Serial.print("Heading error: ");
  Serial.print(purePursuitController.getHeadingError() * 180.0f / M_PI, 1);
  Serial.println(" degrees");
  
  RobotVelocity ppVel = purePursuitController.getLastOutput();
  Serial.print("PP Output: Linear=");
  Serial.print(ppVel.linear, 3);
  Serial.print(" m/s, Angular=");
  Serial.print(ppVel.angular, 3);
  Serial.println(" rad/s");

  } else if (command.startsWith("ppparams ")) {
  // Format: "ppparams max_speed min_speed lookahead"
  int space1 = command.indexOf(' ', 9);
  int space2 = command.indexOf(' ', space1 + 1);
  
  if (space1 > 0 && space2 > 0) {
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
  } else {
    Serial.println("Format: ppparams <max_speed> <min_speed> <lookahead>");
  }
  // === DEBUG COMMANDS ===
  } else if (command == "on") {
    debugMode = true;
    Serial.println("Debug mode enabled");

  } else if (command == "off") {
    debugMode = false;
    Serial.println("Debug mode disabled");
    
  } else if (command == "kintest") {
    WheelRPM rpm = {30.0f, 30.0f};
    RobotVelocity vel = kinematics.forwardKinematics(rpm);
    Serial.print("30 RPM both wheels = ");
    Serial.print(vel.linear, 3);
    Serial.print(" m/s linear, ");
    Serial.print(vel.angular, 3);
    Serial.println(" rad/s angular");
    
  } else if (command == "help") {
    Serial.println("\n=== VELOCITY CONTROL ROBOT ===");
    
    Serial.println("\n--- Motion Control ---");
    Serial.println("vel <lin> <ang> - Set linear/angular velocity");
    Serial.println("stop            - Stop motion");
    
    Serial.println("\n--- PID Tuning ---");
    Serial.println("kp <value>      - Set proportional gain");
    Serial.println("ki <value>      - Set integral gain");
    Serial.println("kd <value>      - Set derivative gain");
    Serial.println("pid <p> <i> <d> - Set all PID values");
    Serial.println("reset           - Reset PID and stop");
    
    Serial.println("\n--- Status & Debug ---");
    Serial.println("status          - Show system status");
    Serial.println("pose            - Show current position");
    Serial.println("resetpose       - Reset position to origin");
    Serial.println("on/off          - Debug mode toggle");
    Serial.println("kintest         - Test kinematics");
    
    Serial.println("\n--- Examples ---");
    Serial.println("vel 0.1 0       - Move forward 0.1 m/s");
    Serial.println("vel 0 0.5       - Rotate 0.5 rad/s");
    Serial.println("vel 0.1 0.3     - Move + rotate");
    Serial.println("pid 2.0 0.1 0.05 - Set PID gains");

    Serial.println("\n--- Pure Pursuit Control ---");
    Serial.println("goto <x> <y> [θ°]  - Move to target position");
    Serial.println("ppstop            - Stop Pure Pursuit");
    Serial.println("ppstatus          - Show PP status");
    Serial.println("ppparams <ms><mn><l> - Set max_speed, min_speed, lookahead");

    Serial.println("\n--- Examples ---");
    Serial.println("goto 0.2 0        - Move 20cm forward");
    Serial.println("goto 0.1 0.1 90   - Move to (10cm,10cm) facing 90°");
    Serial.println("ppparams 0.08 0.01 0.05 - Set PP parameters");
    
  } else if (command.length() > 0) {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}
 
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ROBOT CONTROL SYSTEM ===");

  // Initialize wheels
  if (!wheelLeft.begin()) {
    Serial.println("Failed to initialize left wheel.");
    return;
  } else {
    Serial.println("Left wheel initialized successfully.");
  }

  if (!wheelRight.begin()) {
    Serial.println("Failed to initialize right wheel.");
    return;
  } else {
    Serial.println("Right wheel initialized successfully.");
  }
  
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

  wheelLeft.setDirection(false);
  wheelRight.setDirection(false);

  wheelLeft.setMaxAcceleration(60.0f);
  wheelRight.setMaxAcceleration(60.0f);

  wheelLeft.enableAccelerationLimiting(true);
  wheelRight.enableAccelerationLimiting(true);

  // Setup timer
  static struct repeating_timer timer;
  if (!add_repeating_timer_ms(SAMPLE_TIME_MS, sysUpdateCallback, NULL, &timer)) {
    Serial.println("Failed to add timer");
    return;
  }
  
  Serial.println("Timer started");
  Serial.println("System ready!");
  Serial.println("\nType 'help' for available commands");
  Serial.print("Current PID: KP=");
  Serial.print(KP, 3);
  Serial.print(", KI=");
  Serial.print(KI, 3);
  Serial.print(", KD=");
  Serial.println(KD, 3);
  Serial.println();
}

void loop() {
  // Handle serial commands
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      commandReady = true;
    }
  }
  
  if (commandReady) {
    processCommand(inputString);
    inputString = "";
    commandReady = false;
  }
  
  // Update system components
  if (dataReady) {
    long encoderLeft = wheelLeft.getPulsePosition();
    long encoderRight = wheelRight.getPulsePosition();
    odometry.update(encoderLeft, encoderRight);
    kiddoCar.update();
    Pose2D currentPose = odometry.getPose();
    TargetPoint currentTarget = purePursuitController.getCurrentTarget();
    if (currentTarget.valid && purePursuitController.getState() != PP_GOAL_REACHED) {
      RobotVelocity ppVelocity = purePursuitController.update(currentPose, currentTarget);
      
      // Send PP output to differential drive
      if (purePursuitController.getState() == PP_FOLLOWING) {
        kiddoCar.setVelocity(ppVelocity.linear, ppVelocity.angular);
      }
    }
    purePursuitController.printStatus(debugMode, 25);  // Every 500ms when debug enabled
    dataReady = false;
  }
}