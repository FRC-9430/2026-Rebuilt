package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    public void addVisionMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();

        LimelightHelpers.SetRobotOrientation(
            "limelight", 
            driveState.Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate poseEstimateMT1 = getPoseEstimateMT1();
        LimelightHelpers.PoseEstimate poseEstimateMT2 = getPoseEstimateMT2();

        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimateMT1.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimateMT1.pose.getY());


        boolean doRejectUpdate = false;
        if (poseEstimateMT1.tagCount == 1 && poseEstimateMT1.rawFiducials.length == 1) {
            if (poseEstimateMT1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (poseEstimateMT1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (poseEstimateMT1.tagCount == 0) {
            doRejectUpdate = true;
        }

        Pose2d combPose = new Pose2d(poseEstimateMT1.pose.getTranslation(), poseEstimateMT2.pose.getRotation());


        if (!doRejectUpdate) {
            drivetrain.addVisionMeasurement(
                    poseEstimateMT1.pose,
                    poseEstimateMT1.timestampSeconds);
        }

    }

}
