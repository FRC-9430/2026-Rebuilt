package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {

    private final CommandSwerveDrivetrain drive;
    private final String[] camNames = { "limelight-margo", "limelight-edith", "limelight-agnes" };

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        drive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    }

    @Override
    public void periodic() {
        addVisionMeasurements();
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(camNames[0]);
    }

    public void addVisionMeasurements() {

        var state = drive.getState();

        if (Math.abs(state.Speeds.omegaRadiansPerSecond) > 1.5) {
            return;
        }

        if (Math.abs(state.Speeds.vxMetersPerSecond) > 0.5) {
            return;
        }

        if (Math.abs(state.Speeds.vyMetersPerSecond) > 0.5) {
            return;
        }

        for (int i = 0; i <= 2; i++) {

            LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(camNames[i]);
            logVisionData(est, camNames[i]);

            if (est == null || est.tagCount == 0) {
                continue;
            }

            boolean rejectEstimate = true;

            for (var fiducial : est.rawFiducials) {
                if (fiducial.ambiguity < 0.3) { // Decent Ambiguity required
                    rejectEstimate = false;
                }
            }

            if (rejectEstimate) {
                continue;
            }

            drive.addVisionMeasurement(est.pose, est.timestampSeconds);

            if (Math.abs(state.Speeds.omegaRadiansPerSecond) < 0.05 &&
                    Math.abs(state.Speeds.vxMetersPerSecond) < 0.05 &&
                    Math.abs(state.Speeds.vyMetersPerSecond) < 0.05) { // Robot is still
                for (var fiducial : est.rawFiducials) {
                    if (fiducial.ambiguity < 0.07) { // Low Tag Ambiguity
                        drive.resetRotation(est.pose.getRotation()); // Force Robot Rotation
                        break;
                    }
                }
            }

        }

    }

    private void logVisionData(LimelightHelpers.PoseEstimate est, String camName) {
        SmartDashboard.putNumber("Vision/" + camName + "/Robot Pose Cam Est X", est.pose.getX());
        SmartDashboard.putNumber("Vision/" + camName + "/Robot Pose Cam Est Y", est.pose.getY());
        SmartDashboard.putNumber("Vision/" + camName + "/avgTagDist", est.avgTagDist);
        SmartDashboard.putNumber("Vision/" + camName + "/tagCount", est.tagCount);

        if (est.tagCount == 0)
            return;

        for (var fiducial : est.rawFiducials) {
            SmartDashboard.putNumber("Vision/" + camName + "/Tag/ " + fiducial.id + " Ambiguity", fiducial.ambiguity);
        }

    }

    public void close() {
        drive.close();
    }
}
