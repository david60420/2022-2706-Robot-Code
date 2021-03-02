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
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.config.Config;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.*;
import frc.robot.commands.ramseteAuto.RamseteCommandMerge;
import frc.robot.commands.ramseteAuto.VisionPose;

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

        reverseArmManually = new MoveArmManuallyCommand(-0.35);
        new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(reverseArmManually);

        moveArm = new MoveArmManuallyCommand(10);
        new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(moveArm);

        sensitiveDriving = new SensitiveDriverControl(driverStick);
        new JoystickButton(driverStick, XboxController.Button.kBumperLeft.value).whenHeld(sensitiveDriving);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        int selectorOne = 1;

        if (analogSelectorOne != null){
            selectorOne = analogSelectorOne.getIndex();
            System.out.println("SELECTOR SWITCH NOT NULL AND ID " + selectorOne);
        }
        logger.info("Selectors: " + selectorOne);

        if (Config.hasSelectorSwitches == false) {
            selectorOne = 5;
            logger.info("No Selector Switches - Forced Id: " + selectorOne);
        }

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

            return resetOdometry.andThen(new RamseteCommandMerge(trajectory));

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
                new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajectory1)),
                new RamseteCommandMerge(trajectory2),
                new OuterGoalErrorLoop(true, 3.0),
                new ParallelRaceGroup(new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.5, 8)), new AutoIntakeCommand())
                
            );



        } else if (selectorOne == 6) {
            // Run a example ramsete command
            Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()), DriveBaseHolder.getInstance());

            Trajectory trajDriveForward = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(), 
                new Pose2d(1.5, 0, new Rotation2d(0))), 
                VisionPose.getInstance().getTrajConfig(0, 0, false));

            ParallelRaceGroup cmdGroup = new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajDriveForward));
            return resetOdometry.andThen(cmdGroup);

        } else if (selectorOne == 7) {
            return new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24))));
            // return new OuterGoalErrorLoop(true, 3.0);

        }


        // Also return null if this ever gets to here because safety
        return null;
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