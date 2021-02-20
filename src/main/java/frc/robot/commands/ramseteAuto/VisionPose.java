package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionPose {
    // static variable single_instance of type Singleton 
    private static VisionPose single_instance = null;

    // static method to create instance of Singleton class 
    public static VisionPose getInstance() 
    { 
        if (single_instance == null) 
            single_instance = new VisionPose(); 
  
        return single_instance; 
    }

    VisionData coneMarkerData;
    private VisionPose() {
        coneMarkerData = new VisionData(VisionType.MiddleOfCones, backDriverCamera);
    }

    // Camera Poses from centre
    final Pose2d backDriverCamera = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // <- needs to be filled out


    enum VisionType {
        // 2021 game
        MiddleOfCones(1),
        WallTapeTarget(2), 
        OuterGoal(3),
        TPracticeTarget(4);
        
        

        // 2020 game
        // OuterGoal(4),

        private int value;

        private VisionType(int value) {
            this.value = value;
        }

        private int getValue(VisionType visionType) {
            return value;
        }
    }

    /**
     * Init the Vision 
     */
    public void initVision(VisionType visionType) {
        switch (visionType) {
            case MiddleOfCones:

                break;

            case WallTapeTarget:

                break;
            case OuterGoal:
                VisionCtrlNetTable.setTapeMode(); 
                break;

            case TPracticeTarget:

                break;
            
            
        }
    }

    /**
     * Get the Trajectory Config object needed to construct a trajectory
     * 
     * Velocity Units are in meters per second
     */
    public TrajectoryConfig getTrajConfig(double startVelocity, double endVelocity, boolean isReversed) {
        return Config.trajectoryConfig.setStartVelocity(startVelocity).setEndVelocity(endVelocity).setReversed(isReversed);
    }

    /**
     * Get the Trajectory Config object needed to construct a trajectory
     * 
     * Velocity Units are in meters per second
     * Determines whether to drive in reverse based on Vision Type
     */
    public TrajectoryConfig getTrajConfig(double startVelocity, double endVelocity, VisionType visionType) {
        return getTrajConfig(startVelocity, endVelocity, getReversed(visionType));
    }

    private boolean getReversed(VisionType visionType) {
        switch (visionType) {
        case MiddleOfCones:
            return false;

        case WallTapeTarget:

        case OuterGoal:

        case TPracticeTarget:
            return false;
        }

        return false;
    }

    /**
     * Returns a Pose relative to field of a given vision target
     * 
     * Called Outside of VisionPose
     */
    public Pose2d getTargetPose(VisionType visionType) {
        Pose2d relativePose = targetPose(visionType);

        Pose2d fieldPose = transformPoseToField(relativePose);
        return fieldPose; 
    }

    /**
     * Returns a translation relative to field of a given vision target
     * 
     * Called Outside of VisionPose
     */
    public Translation2d getTargetTranslation(VisionType visionType) {
        Translation2d targetTranslation = targetPose(visionType).getTranslation();

        Translation2d fieldTranslation = transformTranslationToField(targetTranslation);
        return fieldTranslation;
    }

    
    private Pose2d targetPose(VisionType visionType) {
        Pose2d relativePose = null;
        switch (visionType) {
            case MiddleOfCones:
                relativePose = new Pose2d(calcMiddleOfCones(), new Rotation2d(0));
                break;
    
            case WallTapeTarget:
                break;
            case OuterGoal:
                break;

            case TPracticeTarget:
                relativePose = new Pose2d(calcTPracticeTarget(), new Rotation2d(0));
                break;

            }

        
        return relativePose;
    }

    /**
     * Transform Relative Pose with origin at centre of robot to 
     * field Pose with origin of wherever the Odometry reset it's origin to
     */
    private Pose2d transformPoseToField(Pose2d relativePose) {
        Transform2d transformToFieldCoordinateSystem = new Transform2d(new Pose2d(), relativePose);
        return DriveBaseHolder.getInstance().getPose().transformBy(transformToFieldCoordinateSystem); 
    }

    private Translation2d transformTranslationToField(Translation2d relativeTranslation) {
        Rotation2d rotateToField = Rotation2d.fromDegrees(DriveBaseHolder.getInstance().getCurrentAngle() *-1);
        Pose2d odometryPose = DriveBaseHolder.getInstance().getPose();
        Translation2d fieldTranslation = odometryPose.getTranslation().rotateBy(rotateToField);
        return fieldTranslation;
    }

    /**
     * Transform camera location to centre of robot
     */
    private Pose2d transformCameraToCentre(Pose2d relativePose, Pose2d cameraPose, boolean isTranslation) {
        Pose2d centreRelativePose = null;
        if (isTranslation) {
            centreRelativePose = new Pose2d(relativePose.getTranslation().plus(cameraPose.getTranslation()), new Rotation2d(0));
        } else {
            centreRelativePose = relativePose.plus(new Transform2d(cameraPose, new Pose2d()));
        }

        return centreRelativePose;
    }


    /**
     * Transform a pose off a wall
     */
    public Pose2d transformPoseOffWall(Pose2d pose, double distanceOffWall) {
        Rotation2d rotation = pose.getRotation();
        Translation2d offsetTranslation = new Translation2d(rotation.getCos() * distanceOffWall,
                rotation.getSin() * distanceOffWall);
        return pose.transformBy(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    /**
     * Transform a translation off a wall in a certain direction
     */
    public Translation2d transformTranslationOffWall(Translation2d translation, double distanceOffWall, Rotation2d direction) {
        Pose2d pose = transformPoseOffWall(new Pose2d(translation, direction), distanceOffWall);
        return pose.getTranslation();
    }



    private Translation2d calcMiddleOfCones() {
        double distanceToTarget = 0; // Get from vision network table
        double angleAtRobot = 0; // Get from vision network table

        if ((int) distanceToTarget == -99 || (int) angleAtRobot == -99)
            return null;
        if (distanceToTarget <= 0.2 || distanceToTarget > 6.0)
            return null;
        if (Math.abs(angleAtRobot) > 30)
            return null;

        Rotation2d angle = Rotation2d.fromDegrees(angleAtRobot);
        Translation2d translation = new Translation2d(distanceToTarget, angle);

        translation = transformCameraToCentre(new Pose2d(translation, new Rotation2d(0)), coneMarkerData.cameraOffsetFromCentre, true).getTranslation();


        return translation;
    }

    private Translation2d calcTPracticeTarget() {
        double distanceToTarget = 0;
        double angleAtRobot = 0; 

        if ((int) distanceToTarget == -99 || (int) angleAtRobot == -99)
            return null;
        if (distanceToTarget <= 0.2 || distanceToTarget > 6.0)
            return null;
        if (Math.abs(angleAtRobot) > 30)
            return null;

        Rotation2d angle = Rotation2d.fromDegrees(angleAtRobot);
        Translation2d translation = new Translation2d(distanceToTarget, angle);

        translation = transformTranslationOffWall(translation, 3, Rotation2d.fromDegrees(DriveBaseHolder.getInstance().getCurrentAngle()));
    
        return translation;
    }
}
