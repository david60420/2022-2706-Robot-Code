package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.config.Config;

public class PoseScaled extends Pose2d {
    public PoseScaled() {
        super();
    }

    public PoseScaled(double x, double y, double deg) {
        super(x * Config.scaleField, y * Config.scaleField, Rotation2d.fromDegrees(deg));
    }
}
