package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {

    private final CommandSwerveDrivetrain drive;
    private final String[] camNames = {"limelight"};

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
        if (est != null) {
            logVisionData(est);
            drive.addVisionMeasurement(est.pose, est.timestampSeconds);
        }
    }

    private void logVisionData(LimelightHelpers.PoseEstimate est) {
        SmartDashboard.putNumber("Robot Pose Cam Est X", est.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", est.pose.getY());
        SmartDashboard.putNumber("Vision/avgTagDist", est.avgTagDist);
        SmartDashboard.putNumber("Vision/tagCount", est.tagCount);
    }

    public void close() {
        drive.close();
    }
}
