#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
#include "../lib/F3ApplicationLayer/SequenceExecutor/sequenceExecutor.h"
// #include "../lib/F3ApplicationLayer/FuzzyController/fuzzyController.h"  // TODO: Implement
#include <hardware/timer.h>

// === FORWARD DECLARATIONS ===
void handleMotionCommands(String command);
void handlePIDCommands(String command);
void handleSequenceCommands(String command);
void handleOdometryCommands(String command);
void handleStatusCommands(String command);
void handleHelpCommand(String command);

// === HARDWARE LAYER ===
Wheel wheelLeft(
    PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, PWM_FREQ,
    PIN_MOTOR_L_ENCODER_A, PIN_MOTOR_L_ENCODER_B, REDUCER_RATIO, ENCODER_RESOLUTION, X4_MODE,
    KP, KI, KD, SAMPLE_TIME_MS);

Wheel wheelRight(
    PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_FREQ,
    PIN_MOTOR_R_ENCODER_A, PIN_MOTOR_R_ENCODER_B, REDUCER_RATIO, ENCODER_RESOLUTION, X4_MODE,
    KP, KI, KD, SAMPLE_TIME_MS);

// === COMMAND TABLE SYSTEM ===
typedef void (*CommandHandler)(String);

struct CommandEntry
{
  const char *prefix;
  CommandHandler handler;
  bool exactMatch;
  const char *helpText;
};

const CommandEntry COMMAND_TABLE[] = {
    // Motion commands
    {"vel ", handleMotionCommands, false, "Set velocity <lin> <ang> (m/s, rad/s)"},
    {"stop", handleMotionCommands, true, "Emergency stop all motion"},
    {"rawmotor ", handleMotionCommands, false, "Set raw motor speed <left> <right> (-100 to 100)"},

    // PID commands
    {"kp ", handlePIDCommands, false, "Set proportional gain"},
    {"ki ", handlePIDCommands, false, "Set integral gain"},
    {"kd ", handlePIDCommands, false, "Set derivative gain"},
    {"pid ", handlePIDCommands, false, "Set all PID gains <kp> <ki> <kd>"},
    {"pidreset", handlePIDCommands, true, "Reset PID controllers"},

    // Sequence commands
    {"seq ", handleSequenceCommands, false, "Set command sequence (F/L/R)"},
    {"seqstart", handleSequenceCommands, true, "Start sequence execution"},
    {"seqstatus", handleSequenceCommands, true, "Show sequence status"},
    {"seqstop", handleSequenceCommands, true, "Stop sequence execution"},

    // Odometry commands
    {"pose", handleOdometryCommands, true, "Show current robot pose"},
    {"resetpose", handleOdometryCommands, true, "Reset pose to origin"},
    {"setpose ", handleOdometryCommands, false, "Set pose <x> <y> <theta_deg>"},

    // Status/Debug commands
    {"status", handleStatusCommands, true, "Show full system status"},
    {"on", handleStatusCommands, true, "Enable debug mode"},
    {"off", handleStatusCommands, true, "Disable debug mode"},

    // Help command
    {"help", handleHelpCommand, true, "Show this help message"},

    {nullptr, nullptr, false, nullptr}};

// === GLOBAL STATE ===
volatile bool dataReady = false;
volatile bool commandReady = false;
volatile bool debugMode = false;
String inputString = "";

float currentKP = KP;
float currentKI = KI;
float currentKD = KD;

// === APPLICATION LAYER ===
Kinematics kinematics(WHEEL_RADIUS_M, WHEEL_BASE_M);
DifferentialDrive differentialDrive(&wheelLeft, &wheelRight, WHEEL_RADIUS_M, WHEEL_BASE_M, MIN_RPM_PERCENT, MAX_RPM_PERCENT);
Odometry odometry(&kinematics, ENCODER_RESOLUTION, REDUCER_RATIO, X4_MODE);

// === SEQUENCE EXECUTOR ===
SequenceExecutor sequenceExecutor;

// === FUZZY CONTROLLER (TODO) ===
// FuzzyController fuzzyController;

// === CONTROL MODE ===
enum ControlMode
{
  MODE_MANUAL,  // Manual velocity control
  MODE_SEQUENCE // Sequence-based (F/L/R commands)
};

ControlMode currentMode = MODE_MANUAL;
bool pathFollowingActive = false;

// === SYSTEM CALLBACK ===
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

// === SEQUENCE CONTROL ===
void startSequenceMode()
{
  if (sequenceExecutor.getTotalCommands() == 0)
  {
    Serial.println("❌ No sequence set! Use 'seq <commands>' first (e.g., seq FFLR)");
    return;
  }

  currentMode = MODE_SEQUENCE;
  pathFollowingActive = true;

  Serial.println("\n🚀 Sequence mode started with Fuzzy Controller!");
  sequenceExecutor.printStatus();
}

void stopPathFollowing()
{
  pathFollowingActive = false;
  currentMode = MODE_MANUAL;
  differentialDrive.stop();
  Serial.println("⏹ Path following stopped");
}

// === PATH FOLLOWING - SIMPLIFIED FOR FUZZY ===
void updatePathFollowing()
{
  if (!pathFollowingActive)
    return;

  Pose2D currentPose = odometry.getPose();

  // Debug output periodically
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 500)
  {
    lastStatusPrint = millis();
    Serial.print("📍 Pose: (");
    Serial.print(currentPose.x, 3);
    Serial.print(", ");
    Serial.print(currentPose.y, 3);
    Serial.print(", ");
    Serial.print(currentPose.theta * 180.0f / M_PI, 1);
    Serial.println("°)");
  }

  if (currentMode == MODE_SEQUENCE)
  {
    // Check completion
    if (sequenceExecutor.isComplete())
    {
      Serial.println("\n🎉 Sequence completed!");
      stopPathFollowing();
      return;
    }

    // Get current command and target
    char currentCmd = sequenceExecutor.getCurrentCommand();
    PrimitiveRef finalTarget = sequenceExecutor.getFinalTarget();

    // Calculate errors
    float dx = finalTarget.x - currentPose.x;
    float dy = finalTarget.y - currentPose.y;
    float distanceError = sqrt(dx * dx + dy * dy);
    float targetHeading = atan2(dy, dx);
    float headingError = targetHeading - currentPose.theta;

    // Normalize heading error to [-PI, PI]
    while (headingError > M_PI)
      headingError -= 2.0f * M_PI;
    while (headingError < -M_PI)
      headingError += 2.0f * M_PI;

    // TODO: Replace with Fuzzy Controller
    // RobotVelocity velocity = fuzzyController.compute(
    //     distanceError, headingError,
    //     currentPose.x, currentPose.y, currentPose.theta,
    //     currentCmd
    // );

    // TEMPORARY: Simple proportional control
    const float GOAL_TOLERANCE = 0.03f;

    if (distanceError < GOAL_TOLERANCE)
    {
      differentialDrive.stop();

      Serial.print("✓ Command [");
      Serial.print(currentCmd);
      Serial.println("] completed!");

      delay(200);

      if (!sequenceExecutor.forceNextCommand())
      {
        Serial.println("\n🎉 All sequence commands completed!");
        stopPathFollowing();
        return;
      }

      return;
    }

    // Simple control until Fuzzy is implemented
    float linear = min(0.06f, distanceError * 0.8f);
    linear = max(linear, 0.035f);

    float angular = 0.8f * headingError;
    angular = constrain(angular, -0.5f, 0.5f);

    differentialDrive.setVelocity(linear, angular);

    // Debug
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500)
    {
      lastDebug = millis();
      Serial.print("📊 [");
      Serial.print(currentCmd);
      Serial.print("] Dist=");
      Serial.print(distanceError, 3);
      Serial.print("m Heading=");
      Serial.print(headingError * 180.0f / M_PI, 1);
      Serial.print("° Vel=(");
      Serial.print(linear, 3);
      Serial.print(",");
      Serial.print(angular, 3);
      Serial.println(")");
    }
  }
}

// === COMMAND HANDLERS ===
void handleMotionCommands(String command)
{
  if (command.startsWith("vel "))
  {
    int spaceIndex = command.indexOf(' ', 4);
    if (spaceIndex > 0)
    {
      float linear = command.substring(4, spaceIndex).toFloat();
      float angular = command.substring(spaceIndex + 1).toFloat();

      currentMode = MODE_MANUAL;
      pathFollowingActive = false;
      differentialDrive.setVelocity(linear, angular);

      Serial.print("✓ Manual mode: ");
      Serial.print(linear, 3);
      Serial.print(" m/s, ");
      Serial.print(angular, 3);
      Serial.println(" rad/s");
    }
    else
    {
      Serial.println("❌ Format: vel <linear> <angular>");
    }
  }
  else if (command.startsWith("rawmotor "))
  {
    // Format: rawmotor <left_pwm> <right_pwm>
    int spaceIndex = command.indexOf(' ', 9);
    if (spaceIndex > 0)
    {
      float leftPWM = command.substring(9, spaceIndex).toFloat();
      float rightPWM = command.substring(spaceIndex + 1).toFloat();

      // Bypass PID, direct motor control
      wheelLeft.enablePID(false);
      wheelRight.enablePID(false);
      wheelLeft.setRawMotorSpeed(leftPWM);
      wheelRight.setRawMotorSpeed(rightPWM);

      Serial.print("Raw motor: L=");
      Serial.print(leftPWM);
      Serial.print(" R=");
      Serial.println(rightPWM);
    }
  }
  else if (command == "stop")
  {
    stopPathFollowing();
    Serial.println("✓ All motion stopped");
  }
  else
  {
    Serial.println("❌ Unknown motion command");
  }
}

void handlePIDCommands(String command)
{
  if (command.startsWith("kp "))
  {
    currentKP = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KP set to: ");
    Serial.println(currentKP, 4);
  }
  else if (command.startsWith("ki "))
  {
    currentKI = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KI set to: ");
    Serial.println(currentKI, 4);
  }
  else if (command.startsWith("kd "))
  {
    currentKD = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KD set to: ");
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
      Serial.print("✓ PID: KP=");
      Serial.print(currentKP, 4);
      Serial.print(" KI=");
      Serial.print(currentKI, 4);
      Serial.print(" KD=");
      Serial.println(currentKD, 4);
    }
    else
    {
      Serial.println("❌ Format: pid <kp> <ki> <kd>");
    }
  }
  else if (command == "pidreset")
  {
    wheelLeft.resetPID();
    wheelRight.resetPID();
    Serial.println("✓ PID reset");
  }
  else
  {
    Serial.println("❌ Unknown PID command");
  }
}

void handleSequenceCommands(String command)
{
  if (command.startsWith("seq "))
  {
    String commands = command.substring(4);
    commands.toUpperCase();
    sequenceExecutor.setCommandString(commands, 3.0f);

    Serial.print("✓ Sequence set: \"");
    Serial.print(commands);
    Serial.println("\"");
    Serial.println("Type 'seqstart' to begin");
  }
  else if (command == "seqstart")
  {
    sequenceExecutor.start();
    startSequenceMode();
  }
  else if (command == "seqstatus")
  {
    sequenceExecutor.printStatus();
  }
  else if (command == "seqstop")
  {
    sequenceExecutor.stop();
    stopPathFollowing();
  }
  else
  {
    Serial.println("❌ Unknown sequence command");
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
    Serial.println("✓ Pose reset to (0,0,0°)");
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
      Serial.print("✓ Pose set to: (");
      Serial.print(x, 2);
      Serial.print(", ");
      Serial.print(y, 2);
      Serial.print(", ");
      Serial.print(theta_deg, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("❌ Format: setpose <x> <y> <theta_degrees>");
    }
  }
  else
  {
    Serial.println("❌ Unknown odometry command");
  }
}

void handleStatusCommands(String command)
{
  if (command == "status")
  {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║       SYSTEM STATUS                    ║");
    Serial.println("╚════════════════════════════════════════╝");

    Serial.print("Mode: ");
    switch (currentMode)
    {
    case MODE_MANUAL:
      Serial.println("MANUAL");
      break;
    case MODE_SEQUENCE:
      Serial.println("SEQUENCE (Fuzzy)");
      break;
    }
    Serial.print("PID Enabled: ");
    Serial.println(wheelLeft.isPIDEnabled() && wheelRight.isPIDEnabled() ? "YES" : "NO");
    Serial.print("Left RPM: ");
    Serial.print(wheelLeft.getCurrentRPM(), 2);
    Serial.print(", Right RPM: ");
    Serial.println(wheelRight.getCurrentRPM(), 2);

    Serial.print("PID: KP=");
    Serial.print(currentKP, 4);
    Serial.print(" KI=");
    Serial.print(currentKI, 4);
    Serial.print(" KD=");
    Serial.println(currentKD, 4);

    RobotVelocity vel = differentialDrive.getCurrentVelocity();
    Serial.print("Velocity: ");
    Serial.print(vel.linear, 3);
    Serial.print(" m/s, ");
    Serial.print(vel.angular, 3);
    Serial.println(" rad/s");

    Pose2D pose = odometry.getPose();
    Serial.print("Position: (");
    Serial.print(pose.x, 3);
    Serial.print(", ");
    Serial.print(pose.y, 3);
    Serial.print(", ");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println("°)");

    Serial.println("════════════════════════════════════════\n");
  }
  else if (command == "on")
  {
    debugMode = true;
    Serial.println("✓ Debug mode enabled");
  }
  else if (command == "off")
  {
    debugMode = false;
    Serial.println("✓ Debug mode disabled");
  }
  else
  {
    Serial.println("❌ Unknown status command");
  }
}

void handleHelpCommand(String command)
{
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   KIDDOCAR v3.0 - Fuzzy Controller     ║");
  Serial.println("╚════════════════════════════════════════╝");

  Serial.println("\n--- AVAILABLE COMMANDS ---");
  for (int i = 0; COMMAND_TABLE[i].prefix != nullptr; i++)
  {
    Serial.print("  ");
    Serial.print(COMMAND_TABLE[i].prefix);
    int len = strlen(COMMAND_TABLE[i].prefix);
    for (int j = len; j < 16; j++)
      Serial.print(" ");
    Serial.print("- ");
    Serial.println(COMMAND_TABLE[i].helpText);
  }

  Serial.println("\n--- QUICK EXAMPLES ---");
  Serial.println("  seq FFLR          - Set F-F-L-R sequence");
  Serial.println("  seqstart          - Start fuzzy control");
  Serial.println("  vel 0.1 0         - Manual mode");
  Serial.println("  stop              - Emergency stop");
  Serial.println();
}

// === MAIN COMMAND PROCESSOR ===
void processCommand(String command)
{
  command.trim();
  command.toLowerCase();

  if (command.length() == 0)
    return;

  for (int i = 0; COMMAND_TABLE[i].prefix != nullptr; i++)
  {
    bool match = COMMAND_TABLE[i].exactMatch
                     ? (command == COMMAND_TABLE[i].prefix)
                     : command.startsWith(COMMAND_TABLE[i].prefix);

    if (match)
    {
      COMMAND_TABLE[i].handler(command);
      return;
    }
  }

  Serial.println("❌ Unknown command. Type 'help' for available commands.");
}

// === SETUP ===
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   KIDDOCAR v3.0 - Fuzzy Controller     ║");
  Serial.println("╚════════════════════════════════════════╝");

  if (!wheelLeft.begin() || !wheelRight.begin())
  {
    Serial.println("❌ Failed to initialize wheels!");
    while (1)
      ;
  }
  Serial.println("✓ Wheels initialized");

  wheelLeft.setRPMFilter(FilterFactory::createKalman(0.1f, 10.0f));
  wheelRight.setRPMFilter(FilterFactory::createKalman(0.1f, 10.0f));
  Serial.println("✓ Filters configured");

  wheelLeft.setPIDTunings(KP, KI, KD);
  wheelRight.setPIDTunings(KP, KI, KD);
  wheelLeft.enablePID(true);
  wheelRight.enablePID(true);
  wheelLeft.setPIDLimits(-100.0, 100.0);
  wheelRight.setPIDLimits(-100.0, 100.0);
  Serial.println("✓ PID configured");

  wheelLeft.setMaxAcceleration(40.0f);
  wheelRight.setMaxAcceleration(40.0f);
  wheelLeft.enableAccelerationLimiting(true);
  wheelRight.enableAccelerationLimiting(true);
  Serial.println("✓ Acceleration limiting enabled");

  static struct repeating_timer timer;
  if (!add_repeating_timer_ms(SAMPLE_TIME_MS, sysUpdateCallback, NULL, &timer))
  {
    Serial.println("❌ Failed to add timer!");
    while (1)
      ;
  }
  Serial.println("✓ Timer initialized");

  Serial.println("\n✓ System ready! (Fuzzy Controller - TODO)");
  Serial.println("Type 'help' for commands\n");
}

// === MAIN LOOP ===
char inputBuffer[128];
int bufferIndex = 0;

void loop()
{
  // Handle serial input
  while (Serial.available() > 0)
  {
    char inChar = (char)Serial.read();

    if (bufferIndex >= sizeof(inputBuffer) - 1)
    {
      Serial.println("❌ Input too long");
      bufferIndex = 0;
      while (Serial.available())
        Serial.read();
      continue;
    }

    inputBuffer[bufferIndex++] = inChar;

    if (inChar == '\n')
    {
      inputBuffer[bufferIndex] = '\0';
      inputString = String(inputBuffer);
      bufferIndex = 0;
      commandReady = true;
    }
  }

  if (commandReady)
  {
    processCommand(inputString);
    inputString = "";
    commandReady = false;
  }

  if (dataReady)
  {
    long encoderLeft = wheelLeft.getPulsePosition();
    long encoderRight = wheelRight.getPulsePosition();
    odometry.update(encoderLeft, encoderRight);
    differentialDrive.update();

    updatePathFollowing();

    if (debugMode && !pathFollowingActive)
    {
      differentialDrive.printStatus(true, 10);
    }

    dataReady = false;
  }
}