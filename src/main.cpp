#include <Arduino.h>
#include "../lib/F1HardwareLayer/config.h"
#include "../lib/F2ControlLayer/Wheel/wheel.h"
#include "../lib/F3ApplicationLayer/Kinematics/kinematics.h"
#include "../lib/F3ApplicationLayer/DifferentialDrive/differentialDrive.h"
#include "../lib/F3ApplicationLayer/Odometry/odometry.h"
#include "../lib/F3ApplicationLayer/PurePersuit/purePursuit.h"
#include "../lib/F3ApplicationLayer/WaypointManager/waypointManager.h"
#include "../lib/F3ApplicationLayer/SequenceExecutor/sequenceExecutor.h"
#include <hardware/timer.h>

// === FORWARD DECLARATIONS ===
void handleMotionCommands(String command);
void handlePIDCommands(String command);
void handleWaypointCommands(String command);
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

struct CommandEntry {
    const char* prefix;      // Command prefix or exact name
    CommandHandler handler;  // Handler function
    bool exactMatch;         // true = exact match, false = startsWith
    const char* helpText;    // Short description for auto-help
};

// ✅ COMPLETE COMMAND TABLE
const CommandEntry COMMAND_TABLE[] = {
    // Motion commands
    {"vel ",      handleMotionCommands,   false, "Set velocity <lin> <ang> (m/s, rad/s)"},
    {"stop",      handleMotionCommands,   true,  "Emergency stop all motion"},
    
    // PID commands
    {"kp ",       handlePIDCommands,      false, "Set proportional gain"},
    {"ki ",       handlePIDCommands,      false, "Set integral gain"},
    {"kd ",       handlePIDCommands,      false, "Set derivative gain"},
    {"pid ",      handlePIDCommands,      false, "Set all PID gains <kp> <ki> <kd>"},
    {"pidreset",  handlePIDCommands,      true,  "Reset PID controllers"},
    
    // Waypoint commands
    {"pathstraight ", handleWaypointCommands, false, "Create straight path <distance>"},
    {"pathlshape ",   handleWaypointCommands, false, "Create L-shape <leg1> <leg2>"},
    {"pathsquare ",   handleWaypointCommands, false, "Create square path <side>"},
    {"pathcircle ",   handleWaypointCommands, false, "Create circle <radius> [segments]"},
    {"showpath",      handleWaypointCommands, true,  "Display all waypoints"},
    {"clearpath",     handleWaypointCommands, true,  "Clear all waypoints"},
    {"follow",        handleWaypointCommands, true,  "Start waypoint following"},
    
    // Sequence commands
    {"seq ",      handleSequenceCommands, false, "Set command sequence (F/L/R)"},
    {"seqstart",  handleSequenceCommands, true,  "Start sequence execution"},
    {"seqstatus", handleSequenceCommands, true,  "Show sequence status"},
    {"seqstop",   handleSequenceCommands, true,  "Stop sequence execution"},
    
    // Odometry commands
    {"pose",      handleOdometryCommands, true,  "Show current robot pose"},
    {"resetpose", handleOdometryCommands, true,  "Reset pose to origin"},
    {"setpose ",  handleOdometryCommands, false, "Set pose <x> <y> <theta_deg>"},
    
    // Status/Debug commands
    {"status",    handleStatusCommands,   true,  "Show full system status"},
    {"on",        handleStatusCommands,   true,  "Enable debug mode"},
    {"off",       handleStatusCommands,   true,  "Disable debug mode"},
    
    // Help command
    {"help",      handleHelpCommand,      true,  "Show this help message"},
    
    // Sentinel (end of table)
    {nullptr,     nullptr,                false, nullptr}
};

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
DifferentialDrive differentialDrive(&wheelLeft, &wheelRight, WHEEL_RADIUS_M, WHEEL_BASE_M, MAX_RPM);
Odometry odometry(&kinematics, ENCODER_RESOLUTION, REDUCER_RATIO, X4_MODE);

// === PATH FOLLOWING ===
PurePursuitController purePursuit;
WaypointManager waypointManager;
SequenceExecutor sequenceExecutor;

enum ControlMode {
  MODE_MANUAL,   // Manual velocity control
  MODE_WAYPOINT, // Waypoint-based path following
  MODE_SEQUENCE  // Sequence-based (MATLAB style)
};

ControlMode currentMode = MODE_MANUAL;
bool pathFollowingActive = false;
unsigned long lastPPPrintTime = 0;

// === SYSTEM CALLBACK ===
bool sysUpdateCallback(struct repeating_timer *t) {
  if (!dataReady) {
    wheelLeft.update();
    wheelRight.update();
    dataReady = true;
  }
  return true;
}

// === PATH FOLLOWING FUNCTIONS ===
void startWaypointMode() {
  if (waypointManager.isEmpty()) {
    Serial.println("❌ No waypoints! Create path first (pathstraight, pathlshape, pathsquare)");
    return;
  }

  currentMode = MODE_WAYPOINT;
  pathFollowingActive = true;
  waypointManager.reset();

  // Setup Pure Pursuit
  PurePursuitParams params = PurePursuitController::getDefaultParams();
  params.max_speed = 0.08f;
  params.lookahead_dist = 0.10f;
  params.goal_tolerance = 0.02f;
  params.max_angular_vel = 0.5f;
  params.speed_reduction_factor = 0.5f;
  params.enable_fast_math = false;
  purePursuit.setParams(params);

  // Set first waypoint
  TargetPoint firstWaypoint = waypointManager.getCurrentWaypoint();
  Pose2D startPose = odometry.getPose();
  purePursuit.setTargetWithStartPose(firstWaypoint, startPose);

  Serial.println("\n🚀 Waypoint mode started!");
  waypointManager.printCurrentWaypoint();
}

void startSequenceMode() {
  if (!sequenceExecutor.isActive()) {
    Serial.println("❌ No sequence set! Use 'seq <commands>' first (e.g., seq FFLR)");
    return;
  }

  currentMode = MODE_SEQUENCE;
  pathFollowingActive = true;

  // Setup Pure Pursuit
  PurePursuitParams params = PurePursuitController::getDefaultParams();
  params.max_speed = 0.06f;
  params.lookahead_dist = 0.04f;
  params.goal_tolerance = 0.02f;
  params.enable_fast_math = false;
  purePursuit.setParams(params);

  Serial.println("\n🚀 Sequence mode started!");
  sequenceExecutor.printStatus();
}

void stopPathFollowing() {
  pathFollowingActive = false;
  currentMode = MODE_MANUAL;
  differentialDrive.stop();
  purePursuit.reset();
  Serial.println("⏹ Path following stopped");
}

void updatePathFollowing() {
  if (!pathFollowingActive) return;

  Pose2D currentPose = odometry.getPose();

  // Debug output
  static unsigned long lastDebug = 0;
  if (debugMode && millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print("DEBUG Pose: X=");
    Serial.print(currentPose.x, 3);
    Serial.print(" Y=");
    Serial.print(currentPose.y, 3);
    Serial.print(" θ=");
    Serial.print(currentPose.theta * 180.0f / M_PI, 1);
    Serial.print("° | Target: X=");
    Serial.print(waypointManager.getCurrentWaypoint().x, 3);
    Serial.print(" Y=");
    Serial.println(waypointManager.getCurrentWaypoint().y, 3);
  }

  if (currentMode == MODE_WAYPOINT) {
    TargetPoint target = waypointManager.getCurrentWaypoint();

    // Check if straight path
    bool isStraightPath = (abs(target.y - 0.0f) < 0.001f) && 
                          (abs(target.theta) < 0.001f);

    if (isStraightPath) {
      // Simple straight path
      float distance = sqrt((target.x - currentPose.x) * (target.x - currentPose.x) +
                            (target.y - currentPose.y) * (target.y - currentPose.y));

      if (distance < 0.02f) {
        differentialDrive.stop();
        Serial.print("✓ Waypoint [");
        Serial.print(waypointManager.getCurrentIndex());
        Serial.println("] reached!");

        if (waypointManager.moveToNextWaypoint()) {
          TargetPoint nextWaypoint = waypointManager.getCurrentWaypoint();
          purePursuit.setTarget(nextWaypoint);
          waypointManager.printCurrentWaypoint();
        } else {
          Serial.println("\n🎉 All waypoints completed!");
          stopPathFollowing();
        }
      } else {
        differentialDrive.setVelocity(0.08f, 0.0f);
      }
    } else {
      // Complex path - use Pure Pursuit
      RobotVelocity velocity = purePursuit.update(currentPose);
      differentialDrive.setVelocity(velocity);

      if (purePursuit.isGoalReached()) {
        Serial.print("✓ Waypoint [");
        Serial.print(waypointManager.getCurrentIndex());
        Serial.print("] reached! Distance: ");
        Serial.print(purePursuit.getDistanceToTarget(), 3);
        Serial.println("m");

        if (waypointManager.moveToNextWaypoint()) {
          TargetPoint nextWaypoint = waypointManager.getCurrentWaypoint();
          purePursuit.setTarget(nextWaypoint);
          waypointManager.printCurrentWaypoint();
        } else {
          Serial.println("\n🎉 All waypoints completed!");
          stopPathFollowing();
        }
      }
    }

    // Print status
    if (millis() - lastPPPrintTime > 500) {
      if (!isStraightPath) {
        purePursuit.printStatus(true, 1);
      }
      lastPPPrintTime = millis();
    }
  } 
  else if (currentMode == MODE_SEQUENCE) {
    PrimitiveRef ref = sequenceExecutor.update();

    if (sequenceExecutor.isComplete()) {
      Serial.println("\n🎉 Sequence completed!");
      stopPathFollowing();
      return;
    }

    // Convert to TargetPoint
    TargetPoint target = {ref.x, ref.y, ref.theta, true};

    // Update Pure Pursuit only when target changes
    static float lastTargetX = -999.0f;
    static float lastTargetY = -999.0f;
    if (abs(target.x - lastTargetX) > 0.01f || abs(target.y - lastTargetY) > 0.01f) {
      purePursuit.setTarget(target);
      lastTargetX = target.x;
      lastTargetY = target.y;
    }

    RobotVelocity velocity = purePursuit.update(currentPose);
    differentialDrive.setVelocity(velocity);

    // Print status
    if (millis() - lastPPPrintTime > 500) {
      Serial.print("Seq[");
      Serial.print(sequenceExecutor.getCurrentCommand());
      Serial.print("] ");
      purePursuit.printStatus(true, 1);
      lastPPPrintTime = millis();
    }
  }
}

// === COMMAND HANDLERS ===
void handleMotionCommands(String command) {
  if (command.startsWith("vel ")) {
    int spaceIndex = command.indexOf(' ', 4);
    if (spaceIndex > 0) {
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
    } else {
      Serial.println("❌ Format: vel <linear> <angular>");
    }
  }
  else if (command == "stop") {
    stopPathFollowing();
    differentialDrive.stop();
    Serial.println("✓ All motion stopped");
  }
}

void handlePIDCommands(String command) {
  if (command.startsWith("kp ")) {
    currentKP = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KP set to: ");
    Serial.println(currentKP, 4);
  }
  else if (command.startsWith("ki ")) {
    currentKI = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KI set to: ");
    Serial.println(currentKI, 4);
  }
  else if (command.startsWith("kd ")) {
    currentKD = command.substring(3).toFloat();
    wheelLeft.setPIDTunings(currentKP, currentKI, currentKD);
    wheelRight.setPIDTunings(currentKP, currentKI, currentKD);
    Serial.print("✓ KD set to: ");
    Serial.println(currentKD, 4);
  }
  else if (command.startsWith("pid ")) {
    int firstSpace = command.indexOf(' ', 4);
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > 0) {
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
    } else {
      Serial.println("❌ Format: pid <kp> <ki> <kd>");
    }
  }
  else if (command == "pidreset") {
    wheelLeft.resetPID();
    wheelRight.resetPID();
    Serial.println("✓ PID reset");
  }
}

void handleWaypointCommands(String command) {
  if (command.startsWith("pathstraight ")) {
    float distance = command.substring(13).toFloat();
    waypointManager.createStraightPath(distance);
  }
  else if (command.startsWith("pathlshape ")) {
    int spaceIndex = command.indexOf(' ', 11);
    if (spaceIndex > 0) {
      float leg1 = command.substring(11, spaceIndex).toFloat();
      float leg2 = command.substring(spaceIndex + 1).toFloat();
      waypointManager.createLShapePath(leg1, leg2);
    } else {
      Serial.println("❌ Format: pathlshape <leg1> <leg2>");
    }
  }
  else if (command.startsWith("pathsquare ")) {
    float side = command.substring(11).toFloat();
    waypointManager.createSquarePath(side);
  }
  else if (command.startsWith("pathcircle ")) {
    int spaceIndex = command.indexOf(' ', 11);
    float radius = command.substring(11, spaceIndex > 0 ? spaceIndex : command.length()).toFloat();
    int segments = spaceIndex > 0 ? command.substring(spaceIndex + 1).toInt() : 8;
    waypointManager.createCirclePath(radius, segments);
  }
  else if (command == "showpath") {
    waypointManager.printWaypoints();
  }
  else if (command == "clearpath") {
    waypointManager.clear();
    Serial.println("✓ Path cleared");
  }
  else if (command == "follow") {
    startWaypointMode();
  }
}

void handleSequenceCommands(String command) {
  if (command.startsWith("seq ")) {
    String commands = command.substring(4);
    commands.toUpperCase();
    sequenceExecutor.setCommandString(commands, 3.0f);
    sequenceExecutor.start();
    Serial.print("✓ Sequence set: \"");
    Serial.print(commands);
    Serial.println("\"");
    Serial.println("Type 'seqstart' to begin");
  }
  else if (command == "seqstart") {
    startSequenceMode();
  }
  else if (command == "seqstatus") {
    sequenceExecutor.printStatus();
  }
  else if (command == "seqstop") {
    sequenceExecutor.stop();
    stopPathFollowing();
  }
}

void handleOdometryCommands(String command) {
  if (command == "pose") {
    Pose2D pose = odometry.getPose();
    Serial.print("Position: X=");
    Serial.print(pose.x, 3);
    Serial.print("m, Y=");
    Serial.print(pose.y, 3);
    Serial.print("m, Theta=");
    Serial.print(pose.theta * 180.0f / M_PI, 1);
    Serial.println("°");
  }
  else if (command == "resetpose") {
    odometry.resetPose();
    Serial.println("✓ Pose reset to (0,0,0°)");
  }
  else if (command.startsWith("setpose ")) {
    int space1 = command.indexOf(' ', 8);
    int space2 = command.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
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
    } else {
      Serial.println("❌ Format: setpose <x> <y> <theta_degrees>");
    }
  }
}

void handleStatusCommands(String command) {
  if (command == "status") {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println(  "║       SYSTEM STATUS                    ║");
    Serial.println(  "╚════════════════════════════════════════╝");
    
    Serial.print("Mode: ");
    switch (currentMode) {
      case MODE_MANUAL:   Serial.println("MANUAL"); break;
      case MODE_WAYPOINT: Serial.println("WAYPOINT"); break;
      case MODE_SEQUENCE: Serial.println("SEQUENCE"); break;
    }

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

    if (pathFollowingActive) {
      Serial.println("\n--- Path Following: ACTIVE ---");
      purePursuit.printDetailedStatus();
    }
    Serial.println("════════════════════════════════════════\n");
  }
  else if (command == "on") {
    debugMode = true;
    Serial.println("✓ Debug mode enabled");
  }
  else if (command == "off") {
    debugMode = false;
    Serial.println("✓ Debug mode disabled");
  }
}

void handleHelpCommand(String command) {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println(  "║   KIDDOCAR CONTROL SYSTEM v2.0         ║");
  Serial.println(  "╚════════════════════════════════════════╝");

  // Auto-generate help from command table
  Serial.println("\n--- AVAILABLE COMMANDS ---");
  
  const char* lastCategory = "";
  for (int i = 0; COMMAND_TABLE[i].prefix != nullptr; i++) {
    // Group commands by handler (simple categorization)
    const char* cmd = COMMAND_TABLE[i].prefix;
    const char* help = COMMAND_TABLE[i].helpText;
    
    // Print command and help
    Serial.print("  ");
    Serial.print(cmd);
    
    // Padding for alignment
    int len = strlen(cmd);
    for (int j = len; j < 16; j++) Serial.print(" ");
    
    Serial.print("- ");
    Serial.println(help);
  }

  Serial.println("\n--- QUICK EXAMPLES ---");
  Serial.println("  pathsquare 0.3    - Create 30cm square");
  Serial.println("  follow            - Start following");
  Serial.println("  seq FFLR          - Set F-F-L-R sequence");
  Serial.println("  seqstart          - Execute sequence");
  Serial.println("  vel 0.1 0         - Move forward 0.1 m/s");
  Serial.println("  stop              - Emergency stop");
  Serial.println();
}

// === MAIN COMMAND PROCESSOR (TABLE-DRIVEN) ===
void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  if (command.length() == 0) return;
  
  // Search through command table
  for (int i = 0; COMMAND_TABLE[i].prefix != nullptr; i++) {
    bool match = COMMAND_TABLE[i].exactMatch 
        ? (command == COMMAND_TABLE[i].prefix)
        : command.startsWith(COMMAND_TABLE[i].prefix);
    
    if (match) {
      // Found matching command - execute handler
      COMMAND_TABLE[i].handler(command);
      return;
    }
  }
  
  // No match found
  Serial.println("❌ Unknown command. Type 'help' for available commands.");
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println(  "║   KIDDOCAR CONTROL SYSTEM v2.0         ║");
  Serial.println(  "║   with Pure Pursuit Integration        ║");
  Serial.println(  "╚════════════════════════════════════════╝");

  // Initialize wheels
  if (!wheelLeft.begin() || !wheelRight.begin()) {
    Serial.println("❌ Failed to initialize wheels!");
    while (1);
  }
  Serial.println("✓ Wheels initialized");

  // Setup filters
  wheelLeft.setRPMFilter(FilterFactory::createKalman(0.03f, 4.0f));
  wheelRight.setRPMFilter(FilterFactory::createKalman(0.03f, 4.0f));
  Serial.println("✓ Filters configured");

  // Setup PID
  wheelLeft.setPIDTunings(KP, KI, KD);
  wheelRight.setPIDTunings(KP, KI, KD);
  wheelLeft.enablePID(true);
  wheelRight.enablePID(true);
  wheelLeft.setPIDLimits(-100.0, 100.0);
  wheelRight.setPIDLimits(-100.0, 100.0);
  Serial.println("✓ PID configured");

  // Setup acceleration
  wheelLeft.setMaxAcceleration(40.0f);
  wheelRight.setMaxAcceleration(40.0f);
  wheelLeft.enableAccelerationLimiting(true);
  wheelRight.enableAccelerationLimiting(true);
  Serial.println("✓ Acceleration limiting enabled");

  // Setup timer
  static struct repeating_timer timer;
  if (!add_repeating_timer_ms(SAMPLE_TIME_MS, sysUpdateCallback, NULL, &timer)) {
    Serial.println("❌ Failed to add timer!");
    while (1);
  }
  Serial.println("✓ Timer initialized");

  Serial.println("\n✓ System ready!");
  Serial.println("Type 'help' for commands\n");
}

// === MAIN LOOP ===
char inputBuffer[128];
int bufferIndex = 0;

void loop() {
  // Handle serial input
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Check buffer overflow
    if (bufferIndex >= sizeof(inputBuffer) - 1) {
      Serial.println("❌ Input too long, command ignored");
      bufferIndex = 0;
      // Clear remaining input
      while (Serial.available()) Serial.read();
      continue;
    }
    
    inputBuffer[bufferIndex++] = inChar;
    
    if (inChar == '\n') {
      inputBuffer[bufferIndex] = '\0';
      inputString = String(inputBuffer);
      bufferIndex = 0;
      commandReady = true;
    }
  }

  // Process command
  if (commandReady) {
    processCommand(inputString);
    inputString = "";
    commandReady = false;
  }

  // Update system
  if (dataReady) {
    long encoderLeft = wheelLeft.getPulsePosition();
    long encoderRight = wheelRight.getPulsePosition();
    odometry.update(encoderLeft, encoderRight);
    differentialDrive.update();

    // Update path following
    updatePathFollowing();

    // Debug output
    if (debugMode && !pathFollowingActive) {
      odometry.printStatus(true, 10);
    }

    dataReady = false;
  }
}