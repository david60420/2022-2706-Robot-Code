package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase2020;
import frc.robot.subsystems.DriveBasePre2020;
import frc.robot.subsystems.DriveBase;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;


import com.ctre.phoenix.ErrorCode;

/**
 * Config manager for the robot
 */
public class Config {
    
    /**
     * Instructions for set up of robot.conf file on robot
     *
     * 0. Connect to the robot to the robot using a usb cable or the wifi network.
     * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local (ssh admin@roboRIO-2706-FRC.local)
     * a. There is no password on a freshly flashed roboRIO
     * 2. Go up a directory (cd ..)
     * 3. cd into lvuser/ (cd lvuser/)
     * 4. Create a new file called robot.conf (touch robot.conf)
     * 5. Open the file with vi (vi robot.conf)
     * 6. Press i to enter insert mode
     * 7. Add an integer denoting the robot id. If it's the first robot, use 0, second use 1 etc.
     * 8. Press [ESC] followed by typing :wq in order to save and quit
     * 9. To verify this worked type: more robot.conf
     * 10. If it displays the value you entered, it was successful
     * 11. Type exit to safely exit the ssh session
     */
    
    private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");
    
    public static FileHandler logFileHandler;
    
    static {
        try {
            String logFilename = new SimpleDateFormat("'Robotlog_'yyyy'-'MM'-'dd'_'HH'-'mm'-'ss'.txt'").format(new Date());
            logFileHandler = new FileHandler("/home/lvuser/logs/" + logFilename);
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * ID of the robot that code is running on
     */
    public static int robotId = -1;
    
    /**
     * PLACE IDS OF ROBOTS HERE
     **/
    //ID 0 is comp 2020 bot
    //ID 1 is practice 2020 comp bot
    //ID 2 is mini-bot
    //ID 3 is Mergonaut (Deep Space Chassis)
    //
    
    // This is a static class which should not be instantiated
    private Config() {
    
    }
    
    // Static Constants
    private static Class<? extends DriveBase> Pre2020DriveBase = DriveBasePre2020.class.asSubclass(DriveBase.class);
    private static Class<? extends DriveBase> Post2020DriveBase = DriveBase2020.class.asSubclass(DriveBase.class);
    public static Class<? extends DriveBase> DRIVEBASE_CLASS = robotSpecific(Post2020DriveBase, Post2020DriveBase, Post2020DriveBase, Post2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase);
    public static int RIGHT_FRONT_MOTOR = robotSpecific(2, 2, 1, 2, 2);
    public static int RIGHT_REAR_MOTOR = robotSpecific(4, 4, -1, 4, 4);
    public static int LEFT_FRONT_MOTOR = robotSpecific(1, 1, -1, 1, 1);
    public static int LEFT_REAR_MOTOR = robotSpecific(3, 3, 2, 3, 3);
    public static int INTAKE_MOTOR = robotSpecific(6, 6, -1, -1, -1);
    public static int SHOOTER_MOTOR = robotSpecific(5, 5, -1, -1, 16); //protobot is 16
    public static int CLIMBER_TALON = robotSpecific(10, 10, -1, -1, 16);
    public static int AGITATOR_MOTOR = robotSpecific(9, 9, -1, -1);
    public static int RIGHT_MASTER = robotSpecific(RIGHT_FRONT_MOTOR, RIGHT_FRONT_MOTOR, RIGHT_FRONT_MOTOR, RIGHT_FRONT_MOTOR, RIGHT_FRONT_MOTOR);
    public static int LEFT_MASTER = robotSpecific(LEFT_FRONT_MOTOR, LEFT_FRONT_MOTOR, LEFT_REAR_MOTOR, LEFT_FRONT_MOTOR, LEFT_FRONT_MOTOR);


    public static boolean LEFT_SLAVE_ISVICTOR = robotSpecific(true, true, false, false);
    public static boolean RIGHT_SLAVE_ISVICTOR = robotSpecific(true, true, false, false);
    
    // Invert talons to consider forward as forward (same practice for all objects)
    public static boolean LEFT_FRONT_INVERTED = robotSpecific(false, false, false, false);
    public static boolean RIGHT_FRONT_INVERTED = robotSpecific(false, true, true, true);
    public static boolean LEFT_REAR_INVERTED = robotSpecific(false, false, false, false);
    public static boolean RIGHT_REAR_INVERTED = robotSpecific(false, true, false, true);
    public static boolean DRIVETRAIN_LEFT_SENSORPHASE = robotSpecific(false, true, true, true);
    public static boolean DRIVETRAIN_RIGHT_SENSORPHASE = robotSpecific(false, true, true, true);

    public static boolean DRIVETRAIN_INVERT_DIFFERENTIALDRIVE = robotSpecific(false, false, false, false);

    // Current limiter Constants
    public static int PEAK_CURRENT_AMPS = 80;           //Peak current threshold to trigger the current limit
    public static int PEAK_TIME_MS = 250;               //Time after current exceeds peak current to trigger current limit
    public static int CONTIN_CURRENT_AMPS = 40;         //Current to mantain once current limit is triggered 
    public static boolean MOTOR_CURRENT_LIMIT = true;   //Enable or disable motor current limiting.

    public static int TALON_5_PLYBOY = robotSpecific(-1, -1, -1, -1, -1, 5);
    public static int PIGEON_ID = robotSpecific(CLIMBER_TALON, CLIMBER_TALON, 27, LEFT_REAR_MOTOR, LEFT_REAR_MOTOR, TALON_5_PLYBOY);
    
    public static int ANALOG_SELECTOR_ONE = robotSpecific(0, 0, 0, -1, -1, 0);
    
    public static int ARM_TALON = robotSpecific(7, 7, -1, -1);

    public static int FEEDER_SUBSYSTEM_TALON = robotSpecific(8, 8, -1, -1);

    public static int shooterAnalogSensor = robotSpecific(8, 8);

    public static int FEEDER_SWITCH_INPUT = robotSpecific(9, 9, -1, -1);
    public static int FEEDER_SWITCH_OUTPUT = robotSpecific(8, 8, -1, -1);
    public static int FEEDER_MAX_BALLS = 3;
    public static int FEEDERSUBSYSTEM_INDEX_ALLOWABLE_ERROR = 50; 
    public static int FEEDERSUBSYSTEM_POS_PAST_SWITCH = 800;

    public static double FEEDER_MM_CRUISE_VELOCITY = 1500;
    public static double FEEDER_MM_ACCELERATION = 2000;
    public static int FEEDER_MM_SCURVE = 2;

    public static double FEEDERSUBSYSTEM_ARBFF_ONE = 0.13;
    public static double FEEDERSUBSYSTEM_ARBFF_TWO = 0.17;
    public static double FEEDERSUBSYSTEM_ARBFF_THREE = 0.20;
    public static double FEEDERSUBSYSTEM_ARBFF_FOUR = 0.20;
    
    public static Double DRIVE_OPEN_LOOP_DEADBAND = 0.04;
    
    public static Double JOYSTICK_AXIS_DEADBAND = 0.1;
    
    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;
    
    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;
    
    public static boolean INVERT_FIRST_AXIS = robotSpecific(true, true, false);
    public static boolean INVERT_SECOND_AXIS = robotSpecific(false, false, false);
    
    public static boolean HAS_FOLLOWERS = robotSpecific(true, true, false, true, true);

    public static double CONTROLLER_DEADBAND = 0.05;
    
    public static double CURVATURE_OVERRIDE = 0.25;

    public static boolean ARM_PHASE = robotSpecific(true, true, false);
    
    public static boolean INVERT_ARM_TALON = robotSpecific(true, true, false);
    

    public static int ARM_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS = 0;
    
    // Timeouts for sending CAN bus commands
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;

    public static final int TALON_PRIMARY_PID = 0;
    public static final int TALTON_AUXILIARY_PID = 1;

    public static final int DRIVETRAIN_SLOTID_DRIVER = 0;
    
    public static final boolean TELEOP_BRAKE = false;
    
    public static final boolean TELEOP_SQUARE_JOYSTICK_INPUTS = true;
    
    // PIDF values for the arm
    public static double ARM_PID_P = 9.5;//robotSpecific(2); // 5   0.5115
    public static double ARM_PID_I = 0.015;
    public static double ARM_PID_D = 1.0;
    public static double ARM_PID_F = 2.5;//robotSpecific(6); // 0.05  2.0  7
    public static double ARM_PID_IZONE = 20;

    public static int ARM_PID_CRUISE_VELOCITY = robotSpecific(50); //30
    public static int ARM_PID_ACCELERATION = robotSpecific(17); //15
    public static int ARM_PID_SCURVE = robotSpecific(5);

    public static double ARM_PERCENT_AT_HORIZONTAL = robotSpecific(0.0, 0.13);//0.22/1.5);
    public static double ARM_COS_VERT_STRETCH = robotSpecific(0.0, 1.0);//0.6);

    // Define a global constants table for subsystems to use
    public static NetworkTable constantsTable = NetworkTableInstance.getDefault().getTable("constants");

    // Vision Table Constants
    public static String VISION_TABLE_NAME_POWERCELL = "MergeVisionPipelinePi20";
    public static String VISION_TABLE_NAME_OUTERPORT = "MergeVisionPipelinePi21";
    public static String DISTANCE_POWERCELL     = "DistanceToPowerCell";
    public static String DISTANCE_OUTER_PORT    = "DistanceToOuterPort";
    public static String YAW_POWERCELL          = "YawToPowerCell";
    public static String YAW_OUTER_PORT         = "YawToTarget";
    public static String YAW_TO_DIAMOND      = "YawToDiamond";

    // Drivetrain PID values
    public static double DRIVETRAIN_P_SPECIFIC = robotSpecific(0.037, 0.037, 0.0, 0.018d, 0.0, 0.25);
    public static double DRIVETRAIN_D_SPECIFIC = robotSpecific(0.0023, 0.0023, 0.0, 0.0016d, 0.0, 0.03);

    // Drivetain data
    public static double drivetrainWheelDiameter = robotSpecific(0.1524, 0.1524, 0.1016, 0.1524, 0.1524, 0.1524); // Diameter of wheel is 0.1524
    public static int ticksPerRevolution = 4096;

    // Ramsete Default values
    public static double kRamseteB = 2.3;
    public static double kRamseteZeta = 0.73;

    // Frc-characterization data
    // id0: CompBot 
    // id1: PracBot - Previous: 3.24, 0.343 Parking Lot: 1.33, 2.89, 0.164 LargerSpace: 1.15, 2.84, 2
    // id2: Minibot - 
    // id3: DS Robot - Church Parking Lot 1.28, 3.13, 0.463
    public static double ksVolts = robotSpecific(1.1, 1.15, 1.32, 1.28);
    public static double kvVoltSecondsPerMeter = robotSpecific(3.03, 2.84, 4.65, 3.13);
    public static double kaVoltSecondsSquaredPerMeter = robotSpecific(0.4, 0.4, 0.5, 0.463);

    // Track width and kinematics
    public static double kTrackWidth = robotSpecific(0.6, 0.57, 0.3136, 0.569);
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Ramsete Max Velocity and max acceleration
    public static double kMaxSpeedMetersPerSecond = 2.4; // DS Video -> 3.0
    public static double kMaxAccelerationMetersPerSecondSquared = 2.4; // DS Video -> 2.0 

    public static double kRamseteTransferSpeed = kMaxSpeedMetersPerSecond;
    public static double kRamseteTurnAroundSpeed = kMaxSpeedMetersPerSecond; 
    public static double kRamseteBounceEndSpeed = kMaxSpeedMetersPerSecond-0.3;
    public static double kRamseteGalacticSpeed = kMaxSpeedMetersPerSecond-0.7;

    // Converted feet to meters
    public static double METERS_IN_ONE_FOOT = 0.3048;
    // Scale the field
    private static double defaultScale = 1.0;
    public static double scaleField = robotSpecific(defaultScale, defaultScale, 1.0, defaultScale);

    // VISION STUFF BELOW
    // Allowable vision error in meters
    public static double ALLOWABLE_VISION_ODOMETRY_ERROR = 0.5;

    // Vision code that means no target found
    public static double VISION_NO_TARGET_CODE = -99;

    // Change the side of the vision data for each type of angle
    // The desired is to have these as 1 but they are here as backup in case
    // Put either 1 or -1 which gets multiplied by the angle
    public static byte VISION_FLIP_ANGLE = 1;
    public static byte VISION_FLIP_PERPENDICULAR_ANGLE = 1;

    // Set whether to use the vision perpendicular angle or the gyro to figure out 
    // the rotation at a vision target
    public static boolean useVisionPerpendicularAngle = true;
    
    // If camera is facing backwards then put rotation as 180 degrees
    // The location of the camera from the centre of the robot
    public static Pose2d middleOfConesCameraLocation = robotSpecific(
                                            new Pose2d(), 
                                            new Pose2d(), 
                                            new Pose2d(),
                                            new Pose2d());

    // The location of the camera from the centre of the robot
    public static Pose2d diamondTapeCamera = robotSpecific(
                                                new Pose2d(), 
                                                new Pose2d(), 
                                                new Pose2d(),
                                                new Pose2d(0.3, -0.174, Rotation2d.fromDegrees(0)));

    public static final String RELAY_NETWORKTABLE = "ControlRelay";
    public static final int RELAY_RINGLIGHT_REAR_SMALL = 1; // NUMBERS NOT ACCURATELY RELATED TO CAMERAS YET
    public static final int RELAY_RINGLIGHT_REAR_LARGE = 2;
    public static final int RELAY_RINGLIGHT_FRONT = 3;




    // TrajectoryConfig & TrajectoryConstraint - needed to construct a trajectory
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.kDriveKinematics, 10);

    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    
    
    // Ramsete/Talon P values
    // P Values from characterization:
    // id0: 
    // id1:
    // id2:
    // id3: 0.0888 from church parking lot, 0.0105 from basement -> averaged to 0.05 (idk but it worked)
    public static int DRIVETRAIN_SLOTID_RAMSETE = 1;
    public static double RAMSETE_KF = 0;
    public static double RAMSETE_KP = robotSpecific(0.03, 0.0207, 0.0434, 0.05); //0.0884//0.0105
    public static double RAMSETE_KI = 0;
    public static double RAMSETE_KD = 0;
    public static double RAMSETE_ALLOWABLE_PID_ERROR = 0; // <- never stop the P loop from running
    public static double RAMSETE_VOLTAGE_COMPENSATION = 12;

    public static boolean hasSelectorSwitches = robotSpecific(true, false, false, false);

    public static final FluidConstant<Integer> RPM = new FluidConstant<>("Shooter RPM", 1700)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> DRIVETRAIN_P = new FluidConstant<>("DrivetrainP", DRIVETRAIN_P_SPECIFIC)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> DRIVETRAIN_D = new FluidConstant<>("DrivetrainD", DRIVETRAIN_D_SPECIFIC)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> maxTimeOuterPortCommand = new FluidConstant<>("Outer Port Max Time", 1.0)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> maxYawErrorOuterPortCommand = new FluidConstant<>("Outer Port Command Yaw Error", 3.0)
            .registerToTable(Config.constantsTable);
    
    // PID Values for the DrivetrainPIDTurnDelta command
    public static FluidConstant<Double> PIDTURNDELTA_P = new FluidConstant<>("DrivetrainP", 0.018d)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> PIDTURNDELTA_D = new FluidConstant<>("DrivetrainD", 0.0016d)
            .registerToTable(Config.constantsTable);

    // Fluid constant for Drivetrains
    public static FluidConstant<Double> DRIVETRAIN_SENSITIVE_MAX_SPEED = new FluidConstant<>("DrivetrainSensitiveMaxSpeed", 0.2)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> DRIVETRAIN_DEFAULT_MAX_SPEED = new FluidConstant<>("DrivetrainDefaultMaxSpeed", 1.0)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 13000.0)
            .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.8)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.001)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 8.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.2045)
                .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 0.35)
                .registerToTable(Config.constantsTable);

    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IZONE = new FluidConstant<>("FeederSubsystemIZONE", 120)
                .registerToTable(Config.constantsTable);
    
    /**
     * Returns one of the values passed based on the robot ID
     *
     * @param first The first value (default value)
     * @param more  Other values that could be selected
     * @param <T>   The type of the value
     * @return The value selected based on the ID of the robot
     */
    @SafeVarargs
    public static <T> T robotSpecific(T first, T... more) {
        if (getRobotId() < 1 || getRobotId() > more.length) {
            return first;
        } else {
            return more[getRobotId() - 1];
        }
    }
    
    
    /**
     * Obtain the robot id found in the robot.conf file
     *
     * @return The id of the robot
     */
    private static int getRobotId() {
        if (robotId < 0) {
            try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
                robotId = Integer.parseInt(reader.readLine());
            } catch (Exception e) {
                Robot.haltRobot("Can't load Robot ID", e);
            }
        }
        return robotId;
    }
    
    /**
     *
     * @param value The raw axis value from the control stick
     * @return The filtered value defined by the acceptable dead band
     */
    public static double removeJoystickDeadband(double value) {
        if (value <= JOYSTICK_AXIS_DEADBAND && value >= 0) {
            return 0;
        } else if (value >= -JOYSTICK_AXIS_DEADBAND && value <= 0) {
            return 0;
        } else {
            return value;
        }
    }
}
