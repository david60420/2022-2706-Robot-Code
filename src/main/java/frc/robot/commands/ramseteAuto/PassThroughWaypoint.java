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

public class PassThroughWaypoint extends CommandBase {

  private final RamseteCommandMerge ramseteCommand;
  private final VisionType visionType;
  private final double endAfterTime;
  private final Pose2d endPose2d;
  private final double endVelocity;

  /** Creates a new PassThroughWaypoint. */
  public PassThroughWaypoint(RamseteCommandMerge ramseteCommand, VisionType visionType, double endAfterTime, Pose2d endPose2d, double endVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ramseteCommand = ramseteCommand;
    this.visionType = visionType;
    this.endAfterTime = endAfterTime;
    this.endPose2d = endPose2d;
    this.endVelocity = endVelocity;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionPose.getInstance().initVision(visionType);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    VisionPose visionPoseInst = VisionPose.getInstance();
    
    // Ask VisionPose for a translation to the waypoint
    Translation2d waypointTranslation = visionPoseInst.getTargetTranslation(visionType);

    // If the translation isn't null, generate the trajectory
    if(waypointTranslation != null){
      Trajectory trajectory;
      try {
        //Get current robot velocities for left and right sides and find the average
        double[] measuredVelocities = DriveBaseHolder.getInstance().getMeasuredMetersPerSecond();
        double averageCurrentVelocity = (measuredVelocities[0] + measuredVelocities[1])/2.0;

        //Generate trajectory from current pose to endPose, passing through waypointTranslation
        trajectory = TrajectoryGenerator.generateTrajectory(DriveBaseHolder.getInstance().getPose(), 
                                                            List.of(waypointTranslation),
                                                            endPose2d, 
                                                            visionPoseInst.getTrajConfig(averageCurrentVelocity, endVelocity, visionType));

      } catch (Exception e) {
        //TODO: handle exception
      }
    }


    // Generate a trajectory

    // Give the trajectory to ramsete command

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If a certain time has elapsed, stop running the command
    if(ramseteCommand.getElapsedTime() >= endAfterTime){
      return true;
    }
    return false;
  }
}
