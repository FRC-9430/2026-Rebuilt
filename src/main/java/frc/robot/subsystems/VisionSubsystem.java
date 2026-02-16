package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    public void addVisionMeasurements() {
        LimelightHelpers.PoseEstimate poseEstimate = getPoseEstimate();
        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimate.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimate.pose.getY());

        boolean doRejectUpdate = false;
        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
            if (poseEstimate.rawFiducials[0].ambiguity > .5) {
                doRejectUpdate = true;
            }
            if (poseEstimate.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (poseEstimate.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds);
        }

    }

}
