package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    // State for smoothing / gating
    private double m_lastVisionTimestamp = 0.0;
    // minimum time between processing vision updates (seconds)
    private static final double kMinVisionInterval = 0.08;
    // thresholds
    private static final double kMaxAngularRateReject = 1.5; // rad/s (existing)
    private static final double kMaxLinearSpeedReject = 1.0; // m/s (use drivetrain-reported speed)
    private static final double kMaxOutlierDistWhenStopped = 1.0; // meters

    public LimelightHelpers.PoseEstimate getPoseEstimateMT2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    @Override
    public void periodic() {
        addVisionMeasurements();
    }

    public void addVisionMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();

    // compute approximate linear speed from drivetrain reported speeds (more stable)
    double linearSpeed = Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond);
    Pose2d currentOdometryPose = drivetrain.getPose();

        LimelightHelpers.SetRobotOrientation(
                "limelight",
                driveState.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate poseEstimateMT1 = getPoseEstimateMT1();

        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimateMT1.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimateMT1.pose.getY());
        SmartDashboard.putNumber("Vision/avgTagDist", poseEstimateMT1.avgTagDist);
        SmartDashboard.putNumber("Vision/tagCount", poseEstimateMT1.tagCount);

        boolean doRejectUpdate = false;
        if (poseEstimateMT1.tagCount == 1 && poseEstimateMT1.rawFiducials.length == 1) {
            if (poseEstimateMT1.rawFiducials[0].ambiguity > .5) {
                doRejectUpdate = true;
            }
            if (poseEstimateMT1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (poseEstimateMT1.tagCount == 0) {
            doRejectUpdate = true;
        }

        // reject during high angular rates
        if (driveState.Speeds.omegaRadiansPerSecond > kMaxAngularRateReject) {
            doRejectUpdate = true;
        }

        // reject if robot is moving too fast linearly (approximate from odometry)
        if (linearSpeed > kMaxLinearSpeedReject) {
            doRejectUpdate = true;
        }

        // small safeguard: avoid applying vision updates too often
        if (Math.abs(poseEstimateMT1.timestampSeconds - m_lastVisionTimestamp) < kMinVisionInterval) {
            doRejectUpdate = true;
        }

        // Additional outlier rejection: if robot is nearly stopped but vision jump is
        // large,
        // reject the update.
        if (!doRejectUpdate) {
            double distanceToOdometry = Math.hypot(
                    poseEstimateMT1.pose.getX() - currentOdometryPose.getX(),
                    poseEstimateMT1.pose.getY() - currentOdometryPose.getY());

            SmartDashboard.putNumber("Vision/distanceToOdometry", distanceToOdometry);

            if (linearSpeed < kMaxLinearSpeedReject && distanceToOdometry > kMaxOutlierDistWhenStopped
                    && poseEstimateMT1.avgTagDist > 1.0) {
                // large jump while stopped -> likely bad measurement
                doRejectUpdate = true;
            }
        }

        if (!doRejectUpdate) {
            // Compute a per-measurement standard deviation based on ambiguity and distance
            double avgAmbiguity = 0.0;
            if (poseEstimateMT1.rawFiducials.length > 0) {
                for (RawFiducial f : poseEstimateMT1.rawFiducials) {
                    avgAmbiguity += f.ambiguity;
                }
                avgAmbiguity /= poseEstimateMT1.rawFiducials.length;
            }

            // Base sigmas - tune these values as needed
            double sigmaBase = 0.06; // meters
            double sigmaDistFactor = 0.08; // additional meters per meter of distance
            double sigmaAmbigFactor = 0.4; // additional meters per unit ambiguity

            double sigmaXY = sigmaBase + sigmaDistFactor * Math.max(0.0, poseEstimateMT1.avgTagDist)
                    + sigmaAmbigFactor * avgAmbiguity;
            sigmaXY = Math.min(Math.max(sigmaXY, 0.03), 2.0);

            double sigmaTheta = 0.5 * (0.5 + avgAmbiguity); // radians
            sigmaTheta = Math.min(Math.max(sigmaTheta, 0.05), 3.14);

            SmartDashboard.putNumber("Vision/sigmaXY", sigmaXY);
            SmartDashboard.putNumber("Vision/sigmaTheta", sigmaTheta);

            // Build matrix and send measurement with per-measurement std devs
            drivetrain.addVisionMeasurement(
                    poseEstimateMT1.pose,
                    poseEstimateMT1.timestampSeconds,
                    VecBuilder.fill(sigmaXY, sigmaXY, sigmaTheta));

            m_lastVisionTimestamp = poseEstimateMT1.timestampSeconds;
        }

    }

}
