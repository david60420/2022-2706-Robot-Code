package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;;

public class VisionData {
    
    public VisionType visionType;

    public Pose2d cameraOffsetFromCentre;
    


    public VisionData(VisionType visionType, Pose2d cameraLocationFromCentre) {
        this.visionType = visionType;

        cameraOffsetFromCentre = cameraLocationFromCentre;
    }
}
