#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
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
  
  // === INDIVIDUAL WHEEL COMMANDS ===
  // Set left target RPM
  if (command.startsWith("left ")) {
    float rpm = command.substring(5).toFloat();
    wheelLeft.setTargetRPM(rpm);
    Serial.print("Left wheel RPM: ");
    Serial.println(rpm, 2);
  // Set right target RPM
  } else if (command.startsWith("right ")) {
    float rpm = command.substring(6).toFloat();
    wheelRight.setTargetRPM(rpm);
    Serial.print("Right wheel RPM: ");
    Serial.println(rpm, 2);
  // Set both target RPM
  } else if (command.startsWith("both ")) {
    float rpm = command.substring(5).toFloat();
    wheelLeft.setTargetRPM(rpm);
    wheelRight.setTargetRPM(rpm);
    Serial.print("Both wheels RPM: ");
    Serial.println(rpm, 2);
  
  // Stop both wheels
  } else if (command == "stop") {
    wheelLeft.setTargetRPM(0);
    wheelRight.setTargetRPM(0);
    Serial.println("Wheels stopped");
    
  // === MOVEMENT PATTERNS ===
  } else if (command.startsWith("forward ")) {
    float rpm = command.substring(8).toFloat();
    wheelLeft.setTargetRPM(rpm);
    wheelRight.setTargetRPM(rpm);
    Serial.print("Moving forward at ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");
    
  } else if (command.startsWith("backward ")) {
    float rpm = command.substring(9).toFloat();
    wheelLeft.setTargetRPM(-rpm);
    wheelRight.setTargetRPM(-rpm);
    Serial.print("Moving backward at ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");
    
  } else if (command.startsWith("turnleft ")) {
    float rpm = command.substring(9).toFloat();
    wheelLeft.setTargetRPM(-rpm);
    wheelRight.setTargetRPM(rpm);
    Serial.print("Turning left at ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");
    
  } else if (command.startsWith("turnright ")) {
    float rpm = command.substring(10).toFloat();
    wheelLeft.setTargetRPM(rpm);
    wheelRight.setTargetRPM(-rpm);
    Serial.print("Turning right at ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");
    
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
    wheelLeft.setTargetRPM(0);
    wheelRight.setTargetRPM(0);
    Serial.println("PID reset and wheels stopped");
    
  // === STATUS COMMANDS ===
  } else if (command == "status") {
    Serial.println("=== WHEEL STATUS ===");
    Serial.print("Left - Target: ");
    Serial.print(wheelLeft.getTargetRPM(), 2);
    Serial.print(" RPM, Current: ");
    Serial.print(wheelLeft.getCurrentRPM(), 2);
    Serial.println(" RPM");
    
    Serial.print("Right - Target: ");
    Serial.print(wheelRight.getTargetRPM(), 2);
    Serial.print(" RPM, Current: ");
    Serial.print(wheelRight.getCurrentRPM(), 2);
    Serial.println(" RPM");
    
    Serial.print("PID - KP:");
    Serial.print(currentKP, 4);
    Serial.print(" KI:");
    Serial.print(currentKI, 4);
    Serial.print(" KD:");
    Serial.println(currentKD, 4);
    
    if (wheelLeft.getPulsePosition() && wheelRight.getPulsePosition()) {
      Serial.print("Encoders - Left: ");
      Serial.print(wheelLeft.getPulsePosition());
      Serial.print(", Right: ");
      Serial.println(wheelRight.getPulsePosition());
    }
    
  // === DEBUG COMMANDS ===
  } else if (command == "on") {
    debugMode = true;
    Serial.println("Debug mode enabled");

  } else if (command == "off") {
    debugMode = false;
    Serial.println("Debug mode disabled");
    
  } else if (command == "help") {
    Serial.println("\n=== WHEEL TESTING + KINEMATICS ===");
    Serial.println("\n--- Individual Control ---");
    Serial.println("left <rpm>      - Set left wheel RPM");
    Serial.println("right <rpm>     - Set right wheel RPM");
    Serial.println("both <rpm>      - Set both wheels RPM");
    Serial.println("stop            - Stop both wheels");
    
    Serial.println("\n--- Movement Patterns ---");
    Serial.println("forward <rpm>   - Move forward");
    Serial.println("backward <rpm>  - Move backward");
    Serial.println("turnleft <rpm>  - Turn left (differential)");
    Serial.println("turnright <rpm> - Turn right (differential)");
    
    Serial.println("\n--- PID Tuning ---");
    Serial.println("kp <value>      - Set proportional gain");
    Serial.println("ki <value>      - Set integral gain");
    Serial.println("kd <value>      - Set derivative gain");
    Serial.println("pid <p> <i> <d> - Set all PID values");
    Serial.println("reset           - Reset PID and stop");
    
    Serial.println("\n--- Status & Debug ---");
    Serial.println("status          - Show wheel status");
    Serial.println("on/off          - Debug mode toggle");
    
    Serial.println("\n--- Kinematics Test ---");
    Serial.println("kintest         - Test kinematics calculation");
    
    Serial.println("\n--- Examples ---");
    Serial.println("both 30         - Both wheels 30 RPM");
    Serial.println("forward 25      - Forward at 25 RPM");
    Serial.println("turnleft 30     - Turn left at 30 RPM");
    Serial.println("pid 2.0 0.1 0.05 - Set full PID");
    Serial.println("kintest         - Show robot velocity calc");

    Serial.println("\n--- Odometry ---");
    Serial.println("pose            - Show current position");
    Serial.println("resetpose       - Reset to origin");
    Serial.println("setpose <x> <y> <deg> - Set position manually");

  } else if (command == "kintest") {
    WheelRPM rpm = {-30.0f, 30.0f};
    RobotVelocity vel = kinematics.forwardKinematics(rpm);
    Serial.print("30 RPM both wheels = ");
    Serial.print(vel.linear, 3);
    Serial.print(" m/s linear, ");
    Serial.print(vel.angular, 3);
    Serial.println(" rad/s angular");
    Serial.println();
  } else if (command.startsWith("vel ")) {
    int spaceIndex = command.indexOf(' ', 4);

    if (spaceIndex > 0) {
      float linear = command.substring(4, spaceIndex).toFloat();
      float angular = command.substring(spaceIndex + 1).toFloat();
      kiddoCar.setVelocity(linear, angular);
      Serial.println("Velocity set to ");
      Serial.print(linear, 3);
      Serial.print(" m/s linear, ");
      Serial.print(angular, 3);
      Serial.println(" rad/s angular");
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
  } else if (command.length() > 0) {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}
 
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== WHEEL TESTING SYSTEM ===");

  // Initialize wheels
  if (!wheelLeft.begin()) {
    Serial.println("❌ Failed to initialize left wheel.");
    return;
  } else {
    Serial.println("✅ Left wheel initialized successfully.");
  }

  if (!wheelRight.begin()) {
    Serial.println("❌ Failed to initialize right wheel.");
    return;
  } else {
    Serial.println("✅ Right wheel initialized successfully.");
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
    Serial.println("❌ Failed to add timer");
    return;
  }
  
  Serial.println("✅ Timer started (50ms interval)");
  Serial.println("✅ System ready for wheel testing!");
  Serial.println("\nType 'help' for available commands");
  Serial.print("Current PID: KP=");
  Serial.print(KP, 3);
  Serial.print(", KI=");
  Serial.print(KI, 3);
  Serial.print(", KD=");
  Serial.println(KD, 3);
  Serial.println("RPM range: -60 to +60");
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
  
  // Update wheels and print status
  if (dataReady) {
    // wheelLeft.printStatus(debugMode, 10);   // Print every 10 cycles (500ms)
    // wheelRight.printStatus(debugMode, 10);  // Print every 10 cycles (500ms)
    long encoderLeft = wheelLeft.getPulsePosition();
    long encoderRight = wheelRight.getPulsePosition();
    odometry.update(encoderLeft, encoderRight);
    kiddoCar.printStatus(debugMode, 10);    // Print every 10 cycles (500ms)
    dataReady = false;
  }
}