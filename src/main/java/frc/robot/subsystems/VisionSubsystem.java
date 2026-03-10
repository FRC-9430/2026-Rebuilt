package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {

    private final CommandSwerveDrivetrain drive;
    private final String[] camNames = { "limelight" };

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        addVisionMeasurements();
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(camNames[0]);
    }

    public void addVisionMeasurements() {

        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(camNames[0]);
        logVisionData(est);

        if (est == null || est.tagCount == 0) {
            return;
        }

        if (est.avgTagDist > 4) {
            return;
        }

        drive.addVisionMeasurement(est.pose, est.timestampSeconds);
    }

    private void logVisionData(LimelightHelpers.PoseEstimate est) {
        SmartDashboard.putNumber("Vision/Robot Pose Cam Est X", est.pose.getX());
        SmartDashboard.putNumber("Vision/Robot Pose Cam Est Y", est.pose.getY());
        SmartDashboard.putNumber("Vision/avgTagDist", est.avgTagDist);
        SmartDashboard.putNumber("Vision/tagCount", est.tagCount);
        if (est.tagCount == 0)
            return;

        for (var fiducial : est.rawFiducials) {
            SmartDashboard.putNumber("Vision/Tag/ " + fiducial.id + " Ambiguity", fiducial.ambiguity);
        }

    }

    public void close() {
        drive.close();
    }
}
