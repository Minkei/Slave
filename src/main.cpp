#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
#include "../lib/F3ApplicationLayer/MotionController/motionController.h"
#include <hardware/timer.h>

// Wheel objects
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

// Motion Controller
MotionController motionController(&kiddoCar, &odometry, 200.0f);

// Timer callback
bool sysUpdateCallback(struct repeating_timer *t) {
  if (!dataReady) {
    wheelLeft.update();
    wheelRight.update();
    kiddoCar.update();
    
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
  } else if (command.startsWith("kp ")) {
    currentKP = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KP set to: ");
    Serial.println(currentKP, 4);
    
  } else if (command.startsWith("ki ")) {
    currentKI = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KI set to: ");
    Serial.println(currentKI, 4);
    
  } else if (command.startsWith("kd ")) {
    currentKD = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("KD set to: ");
    Serial.println(currentKD, 4);
    
  } else if (command.startsWith("pid ")) {
    // Format: "pid kp ki kd"
    int firstSpace = command.indexOf(' ', 4);
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
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
    } else {
      Serial.println("Format: pid <kp> <ki> <kd>");
    }
    
  } else if (command == "reset") {
    wheelLeft.resetPID();
    wheelRight.resetPID();
    kiddoCar.setVelocity(0, 0);
    Serial.println("PID reset and motion stopped");
    
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
    
  // === ODOMETRY COMMANDS ===
  } else if (command == "pose") {
    Pose2D pose = odometry.getPose();
    Serial.print("Position: X=");
    Serial.print(pose.x, 3);
    Serial.print("m, Y=");
    Serial.print(pose.y, 3);
    Serial.print("m, Theta=");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println("°");
      
  } else if (command == "resetpose") {
    odometry.resetPose();
    Serial.println("Pose reset to (0,0,0°)");
      
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
      Serial.println("°)");
    } else {
      Serial.println("Format: setpose <x> <y> <theta_degrees>");
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
    
    // Debug output
    odometry.printStatus(debugMode, 25);  // Every 500ms when debug enabled
    
    dataReady = false;
  }
}