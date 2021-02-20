// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;

public class PassThroughWaypoint extends CommandBase {

  private final RamseteCommandMerge ramseteCommand;
  private final VisionType visionType;
  private final double endAfterTime;
  private final Pose2d endPose2d;

  /** Creates a new PassThroughWaypoint. */
  public PassThroughWaypoint(RamseteCommandMerge ramseteCommand, VisionType visionType, double endAfterTime, Pose2d endPose2d) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ramseteCommand = ramseteCommand;
    this.visionType = visionType;
    this.endAfterTime = endAfterTime;
    this.endPose2d = endPose2d;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionPose.getInstance().initVision(visionType);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
