package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {

    private final CommandSwerveDrivetrain drive;
    private final String[] camNames = { "limelight" };

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

    /**
     * Adds vision measurements to the drivetrain's pose estimation system.
     * The estimation from the Limelight camera is used to correct the robot's pose
     * if certain conditions are met, such as low ambiguity in detected fiducials
     * and the robot being relatively still.
     */
    public void addVisionMeasurements() {

        var state = drive.getState();

        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(camNames[0]);
        logVisionData(est);

        if (est == null || est.tagCount == 0 || state.Speeds.omegaRadiansPerSecond > 30) {
            return;
        }

        for (var fiducial : est.rawFiducials) {
            if (fiducial.ambiguity < 0.3) { // Decent Abmiguity required
                break;
            }
            return;
        }

        if (Math.abs(state.Speeds.omegaRadiansPerSecond) > 1.5) {
            return;
        }

        if (Math.abs(est.pose.getRotation().getDegrees() - state.Pose.getRotation().getDegrees()) > 1) {
            // If estimated position is greater than 1 degree off from detected
            if (Math.abs(state.Speeds.omegaRadiansPerSecond) < 0.05 &&
                    Math.abs(state.Speeds.vxMetersPerSecond) < 0.05 &&
                    Math.abs(state.Speeds.vyMetersPerSecond) < 0.05) { // Robot is still
                for (var fiducial : est.rawFiducials) {
                    if (fiducial.ambiguity < 0.07) { // Low Tag Ambiguity
                        drive.resetRotation(est.pose.getRotation()); // Force Robot Rotation
                        drive.resetPose(est.pose); // Force Robot Pose
                        break;
                    }
                }
            }
        }

        drive.addVisionMeasurement(est.pose, est.timestampSeconds);

    }

    /**
     * Logs vision data to the SmartDashboard for debugging purposes.
     * 
     * @param est The pose estimate from the Limelight camera, containing
     *            information about the robot's position and detected fiducials.
     */
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
