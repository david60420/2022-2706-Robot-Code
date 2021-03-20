/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.config.Config;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.*;
import frc.robot.commands.ramseteAuto.DriveToWaypoint;
import frc.robot.commands.ramseteAuto.PassThroughWaypoint;
import frc.robot.commands.ramseteAuto.PoseScaled;
import frc.robot.commands.ramseteAuto.RamseteCommandMerge;
import frc.robot.commands.ramseteAuto.TranslationScaled;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;

import java.util.List;
import java.util.logging.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    
    // RobotContainer is a singleton class
    private static RobotContainer currentInstance;

  // The robot's subsystems and commands are defined here...    
  private Joystick driverStick;
  private Joystick controlStick;
  public AnalogSelector analogSelectorOne;
  private AnalogSelector analogSelectorTwo;
  private Command driveCommand;
  private Command intakeCommand;
  private Command reverseFeeder;
  private Command moveToOuterPort;
    private Command reverseArmManually;
  private Command positionPowercell;
  private Command rampShooterCommand;
  private Command incrementFeeder;
  private Command moveArm;
  private Command sensitiveDriving;
  private Logger logger = Logger.getLogger("RobotContainer");
  private final double AUTO_DRIVE_TIME = 1.0;
  private final double AUTO_LEFT_MOTOR_SPEED = 0.2;
  private final double AUTO_RIGHT_MOTOR_SPEED = 0.2;
    private Command runFeeder;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        logger.addHandler(Config.logFileHandler);
        if (Config.ANALOG_SELECTOR_ONE != -1) {
            analogSelectorOne = new AnalogSelector(Config.ANALOG_SELECTOR_ONE);
        }

        ArmSubsystem armSubsystem;
        if (Config.ARM_TALON != -1)
            armSubsystem = ArmSubsystem.getInstance();

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverStick = new Joystick(0);
        controlStick = new Joystick(1);
      
        // Instantiate the intake command and bind it
        intakeCommand = new OperatorIntakeCommand();
        new JoystickButton(controlStick, XboxController.Button.kBumperLeft.value).whenHeld(intakeCommand);

        reverseFeeder = new ReverseFeeder();
        new JoystickButton(controlStick, XboxController.Button.kB.value).whenHeld(reverseFeeder);

        runFeeder = new RunFeederCommand(-0.3);
        new JoystickButton(controlStick, XboxController.Button.kY.value).whenHeld(runFeeder);

        incrementFeeder = new IncrementFeeder(-FeederSubsystem.FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
        new JoystickButton(controlStick, XboxController.Button.kX.value).whenHeld(incrementFeeder);

        rampShooterCommand = new SpinUpShooter();
        new JoystickButton(controlStick, XboxController.Button.kA.value).toggleWhenActive(rampShooterCommand);

        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, true);
        DriveBaseHolder.getInstance().setDefaultCommand(driveCommand);

        positionPowercell = new PositionPowercellCommand();
        new JoystickButton(controlStick, XboxController.Button.kBumperRight.value).toggleWhenActive(positionPowercell, true);

        moveToOuterPort = new TurnToOuterPortCommand(true, 3.0, 2.0);
        new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(moveToOuterPort, true);

        if (Config.ARM_TALON != -1) {
            reverseArmManually = new MoveArmManuallyCommand(-0.35);
            new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(reverseArmManually);

            moveArm = new MoveArmManuallyCommand(10);
            new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(moveArm);
        }

        sensitiveDriving = new SensitiveDriverControl(driverStick);
        new JoystickButton(driverStick, XboxController.Button.kBumperLeft.value).whenHeld(sensitiveDriving);

        Command resetHeading = new InstantCommand(() -> DriveBaseHolder.getInstance().resetHeading(Rotation2d.fromDegrees(0)));
        new JoystickButton(driverStick, XboxController.Button.kStart.value).whenActive(resetHeading);

        if (Config.FEEDER_SUBSYSTEM_TALON != -1) {
            // Set default command of feeder to index when limit is pressed
            Command indexFeeder = new IndexBall().andThen(new DoNothingForSeconds(1.5));
            Command pollInputSwitch = new PollLimitSwitch(indexFeeder, FeederSubsystem.getInstance(), FeederSubsystem::isBallAtInput);
            FeederSubsystem.getInstance().setDefaultCommand(pollInputSwitch); 
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Testing forced numbers
        int selectFolder = 2;
        int selectPath = 3;

        int selectorOne = 0;

        if (analogSelectorOne != null){
            selectorOne = analogSelectorOne.getIndex();
            System.out.println("SELECTOR SWITCH NOT NULL AND ID " + selectorOne);
        }
        logger.info("Selectors: " + selectorOne);

        if (Config.hasSelectorSwitches == false) {
            selectorOne = selectPath;
            logger.info("No Selector Switches - Forced Id: " + selectorOne);
        }
        
        switch (selectFolder) {
            case 1:
                return getAutoCommandTest(selectorOne);

            case 2:
                return getAutoCommandIRAH(selectorOne);
        }

        return null;
    }

    private Command getAutoCommandTest(int selectorOne) {

        if (selectorOne == 0) {
            // This is our 'do nothing' selector
            return null;
        } else if (selectorOne == 1) {
            return new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.7, 7)); //.andThen(new DriveWithTime(0.5, 0.5, 0.5));
           // return new DriveWithTime(AUTO_DRIVE_TIME,  AUTO_LEFT_MOTOR_SPEED,  AUTO_RIGHT_MOTOR_SPEED);
        
        } else if(selectorOne == 2) {
            return new DriveWithTime(0.5, 0.5, 0.5); 

        } else if(selectorOne == 3) {
            // Directly Tell the talons to go both sides a specific value. (For setting inversions)
            SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter);
            double vel = 2.0;

            return new RunCommand(() -> DriveBaseHolder.getInstance().tankDriveVelocities(vel, vel, feedforward.calculate(vel), feedforward.calculate(vel)));

        } else if(selectorOne == 4) {
            // Run a example ramsete command
            Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()), DriveBaseHolder.getInstance());
            
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(), new Pose2d(2.5, -0.5, Rotation2d.fromDegrees(0))), 
                Config.trajectoryConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false));

            return resetOdometry.andThen(new RamseteCommandMerge(trajectory, "R4-SingleTraj"));

        } else if (selectorOne == 5) {

            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24)), // START POSE
                new Pose2d(5.2, 1.2, Rotation2d.fromDegrees(0))),  // END POSE
                VisionPose.getInstance().getTrajConfig(0, 2, false)); // CONFIG

            Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.2, 1.2, Rotation2d.fromDegrees(0)), // START POSE
                List.of(new Translation2d(3.5, 1.5)), // WAYPOINT
                new Pose2d(3.0, 2.1, Rotation2d.fromDegrees(-10)),  // END POSE
                VisionPose.getInstance().getTrajConfig(0, 0, true)); // CONFIG


            return new SequentialCommandGroup(
                new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24)))),
                new InstantCommand(() -> FeederSubsystem.getInstance().setBallsAroundFeeder(0)),
                new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajectory1, "R5FullR-1")),
                new RamseteCommandMerge(trajectory2, "R5FullR-2").alongWith(new IndexBall()),
                new OuterGoalErrorLoop(true, 3.0),
                new ParallelRaceGroup(new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.7, 8)), new AutoIntakeCommand())
            );



        } else if (selectorOne == 6) {
            // Run a example ramsete command
            Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()), DriveBaseHolder.getInstance());

            Trajectory trajDriveForward = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(), 
                new Pose2d(1.5, 0, new Rotation2d(0))), 
                VisionPose.getInstance().getTrajConfig(0, 0, false));

            ParallelRaceGroup cmdGroup = new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajDriveForward, "R6DriveForward"));
            return resetOdometry.andThen(cmdGroup);

        } else if (selectorOne == 7) {
            return new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24))));
            // return new OuterGoalErrorLoop(true, 3.0);

        } else if (selectorOne == 8) {
            return new InstantCommand(() -> FeederSubsystem.getInstance().setBallsAroundFeeder(0));
        } else if (selectorOne == 9) {

            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // A
                new Pose2d(1.7, 0, Rotation2d.fromDegrees(0))), // B
                VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(1.7, 0, Rotation2d.fromDegrees(0)), // B
                new Pose2d(0.5, -0.72, Rotation2d.fromDegrees(90)), // B'
                new Pose2d(1.29, -1.3, Rotation2d.fromDegrees(171))),  // C
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );

            Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(1.29, -1.3, Rotation2d.fromDegrees(171)), // C
                new Pose2d(0.0, -0.3, Rotation2d.fromDegrees(180)), // C'
                new Pose2d(-1.5, 0.05, Rotation2d.fromDegrees(160))), // D
                new TrajectoryConfig(Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Config.kDriveKinematics).addConstraint(Config.autoVoltageConstraint)
                    .addConstraint(new RectangularRegionConstraint(new Translation2d(-3, 0.5), new Translation2d(-0.5, -2.0), new MaxVelocityConstraint(0.8)))   // VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-1.5, 0.05, Rotation2d.fromDegrees(160)), // D
                new Pose2d(-0.62, -0.16, Rotation2d.fromDegrees(-150))), // E
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );
            
            Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-0.62, -0.16, Rotation2d.fromDegrees(-150)),
                new Pose2d(-1.48, -0.38, Rotation2d.fromDegrees(-160))),
                VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-1.48, -0.38, Rotation2d.fromDegrees(-160)),
                new Pose2d(-0.34, -0.52, Rotation2d.fromDegrees(150))),
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );

            return new SequentialCommandGroup(
                new InstantCommand(DriveBaseHolder.getInstance()::setBrakeMode),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory1, "IRAHr2-T1"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory2, "IRAHr2-T2"),
                new OuterGoalErrorLoop(true, 3.0),
                new SpinUpShooterWithTime((int) Config.RPM.get()+300, 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory3, "IRAHr3-T2"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory4, "IRAHr4-T2"),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory5, "IRAHr5-T2"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory6, "IRAHr6-T2"),

                new OuterGoalErrorLoop(true, 3.0),
                new SpinUpShooterWithTime((int) Config.RPM.get()+700, 7).alongWith(new RunFeederCommandWithTime(-0.5, 7))
            );

        } else if (selectorOne == 10) {
            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(), 
                List.of(new Translation2d(1, -0.4)),
                new Pose2d(1.5, -1.7, Rotation2d.fromDegrees(-45)),
                VisionPose.getInstance().getTrajConfig(0, 0, VisionPose.VisionType.TPracticeTarget));

            RamseteCommandMerge ramsete = new RamseteCommandMerge(trajectory1, "PassThruWaypointTest");

            // return ramsete;
            return new ParallelCommandGroup(ramsete,
                    new PassThroughWaypoint(ramsete, VisionPose.VisionType.TPracticeTarget, 8, trajectory1.sample(trajectory1.getTotalTimeSeconds()).poseMeters, 0, 0.5),
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()))); 
        }


        // Also return null if this ever gets to here because safety
        return null;
    }

    private Command getAutoCommandIRAH(int selectorOne) {
        switch (selectorOne) {

            case 0:
                return null;

            case 1:{
                /** Bounce path  */
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(),
                    new PoseScaled()),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTurnAroundSpeed, true));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Bounce-P1");

                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                    endPose(trajectory1),
                    List.of(new TranslationScaled()),
                    new PoseScaled(),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Bounce-P2");

                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory2),
                    new PoseScaled()),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTurnAroundSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Bounce-P3");

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                    endPose(trajectory3), List.of(
                    new TranslationScaled(0, 0), 
                    new TranslationScaled(0, 0), 
                    new TranslationScaled(0, 0)),
                    new PoseScaled(),
                    VisionPose.getInstance().getTrajConfig(0, 0, VisionType.DiamondTape));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Bounce-P4");

                Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory4),
                    new PoseScaled()),
                    VisionPose.getInstance().getTrajConfig(0, 0, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Bounce-P5");

                double waypointRadiusMeters = 0.5;

                return new SequentialCommandGroup(
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                    ramsete1,
                    new ParallelRaceGroup(ramsete2, new PassThroughWaypoint(ramsete2, VisionType.MiddleOfCones, 6, ramsete2.getTargetPose(), Config.kRamseteTransferSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete3, new PassThroughWaypoint(ramsete3, VisionType.MiddleOfCones, 6, ramsete3.getTargetPose(), Config.kRamseteTurnAroundSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete4, new DriveToWaypoint(ramsete4, VisionType.DiamondTape, 10, Config.kRamseteTurnAroundSpeed)),
                    new ParallelRaceGroup(ramsete5, new PassThroughWaypoint(ramsete5, VisionType.MiddleOfCones, 6, ramsete4.getTargetPose(), 0, waypointRadiusMeters))
                );
            }
            case 2:{
                // Barrel Racing path

                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new PoseScaled(0.922, -2.357, 0.0), 
                  List.of(
                    new TranslationScaled(),
                    new TranslationScaled(),
                    new TranslationScaled(),
                    new TranslationScaled() ),
                    new PoseScaled(),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, VisionType.DiamondTape));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Barrel-P1");
                
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of( 
                    new PoseScaled(), 
                    new PoseScaled()), 
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.DiamondTape));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Barrel-P2");

                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                  List.of(
                    new TranslationScaled(),
                    new TranslationScaled(),
                    new TranslationScaled()),
                    new PoseScaled(),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.DiamondTape));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Barrel-P3");
                

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of( 
                    new PoseScaled(), 
                    new PoseScaled()), 
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.DiamondTape));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Barrel-P4");

                Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                  List.of(
                    new TranslationScaled(),
                    new TranslationScaled(),
                    new TranslationScaled()),
                    new PoseScaled(),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Barrel-P5");
            
            }
            
        }

        // If nothing runs do nothing
        return null;   

    }

    /**
     * Helper method for constructing trajectories. Gets the final pose of a given trajectory.
     * 
     * RamseteCommandMerge has the same functionality with the getTargetPose() method
     * 
     * @param trajectory A given trajectory to find what pose it will end at.
     * @return The end pose.
     */
    private Pose2d endPose(Trajectory trajectory) {
        return trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
    }

    public void joystickRumble(double leftValue, double rightValue) {
        //Joystick rumble (driver feedback). leftValue/rightValue sets vibration force.
        driverStick.setRumble(RumbleType.kLeftRumble, leftValue);
        driverStick.setRumble(RumbleType.kRightRumble, rightValue);
    }

    /**
     * Initialize the current RobotContainer instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new RobotContainer();
        }
    }

    public static RobotContainer getInstance() {
        init();
        return currentInstance;
    }
    
    
}