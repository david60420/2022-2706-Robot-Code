// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import java.util.logging.Logger;
import frc.robot.config.Config;

public class DriveToWaypoint extends CommandBase {

  private final RamseteCommandMerge ramseteCommand;
  private final VisionType visionType;
  private final double endAfterTime;
  private final double endVelocity;
  private int frequency; //The command will calculate a new trajectory every x cycles 
  private int cyclesToRecalculation;

  // Logging
  private Logger logger = Logger.getLogger("DriveToWaypoint");

  /** Creates a new DriveToWaypoint. */
  public DriveToWaypoint(RamseteCommandMerge ramseteCommand, VisionType visionType, double endAfterTime, double endVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ramseteCommand = ramseteCommand;
    this.visionType = visionType;
    this.endAfterTime = endAfterTime;
    this.endVelocity = endVelocity;

    logger.addHandler(Config.logFileHandler);
    this.frequency = 5;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionPose.getInstance().initVision(visionType);
    cyclesToRecalculation = frequency - 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(cyclesToRecalculation == 0){
      cyclesToRecalculation = frequency - 1;

      VisionPose visionPoseInst = VisionPose.getInstance();
      
      // Ask VisionPose for the pose of the endpoint
      Pose2d endPose2d = visionPoseInst.getTargetPose(visionType);

      // If the translation isn't null, generate the trajectory
      if(endPose2d != null){
        Trajectory trajectory;
        try {
          //Get current robot velocities for left and right sides and find the average
          double[] measuredVelocities = DriveBaseHolder.getInstance().getMeasuredMetersPerSecond();
          double averageCurrentVelocity = (measuredVelocities[0] + measuredVelocities[1])/2.0;

          //Generate trajectory from current pose to endPose
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(DriveBaseHolder.getInstance().getPose(), endPose2d),
                                                              visionPoseInst.getTrajConfig(averageCurrentVelocity, endVelocity, visionType));

        } catch (Exception e) {
          logger.severe("Trajectory Generator failed to calculate a valid trajectory: " + e.getMessage());
          return;
        }

        if(trajectory != null){
          //Give the ramsete command the updated trajectory
          ramseteCommand.setNewTrajectory(trajectory);
          System.out.println("Endpoint Pose is " + endPose2d.toString());
        }

      }
    }
    else{
      cyclesToRecalculation--;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If a certain time has elapsed, stop running the command
    if(ramseteCommand.getElapsedTime() >= endAfterTime){
      return true;
    }
    return false;
  }

  //Set recaclulation frequency to new value
  public void setFrequency(int frequency){
    this.frequency = frequency;
  }

}
