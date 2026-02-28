package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable{

    public final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
            kDefaultSigmaX,
            kDefaultSigmaY,
            kDefaultSigmaTheta
        ));

        // Mode 1: IMU data is used for MegaTag2 orientation
        LimelightHelpers.SetIMUMode("limelight", kLimelightIMUMode);
    }

    // state management
    private double m_lastVisionTimestamp = 0.0;

    // Tiered Trust Model Constants
    /** TODO 20260225.1838 bbontrager, where do constants go? why don't they go there?
     * Vision Input Trust Factors
     * Tier 1 "High"        : Multi-tag or close single-tag (< 3m).
     * Tier 2 "OK"          : Multi-tag at medium range (3m - 4m).
     * Tier 3 "Dubious"     : Multi-tag at long range (> 4m) — requires 3
     *                        consecutive frames and high sigmas.
     * Tier 4 "Rejected"    : Single tag > 3m.
     */

    // Rejection Thresholds (Gating)
    private static final double kMinVisionInterval = 0.08; // seconds
    private static final double kMaxAngularRateReject = 1.5; // rad/s (existing)
    private static final double kMaxLinearSpeedReject = 1.0; // m/s (use drivetrain-reported speed)
    private static final double kMaxOutlierDistWhenStopped = 1.0; // meters
    public static final double kMaxAmbiguity = 0.5;
    public static final double kMaxSingleTagDist = 3.0; // meters (Tier 4)
    private static final double kMinDistForOutlierReject = 1.0; // meters
    private static final int kLimelightIMUMode = 1;

    // Base sigmas
    public static final double kSigmaBase = 0.06; // meters
    public static final double kSigmaDistFactor = 0.08; // additional meters per meter of distance
    public static final double kSigmaAmbigFactor = 0.4; // additional meters per unit ambiguity

    // Sigma clamping bounds
    public static final double kMinSigmaXY = 0.03; // meters (prevents overconfidence)
    public static final double kMaxSigmaXY = 2.0;  // meters (prevents mathematical instability)

    // Sigma constants (radians)
    public static final double kSigmaThetaBase = 0.25; // radians (0.5 * 0.5)
    public static final double kSigmaThetaAmbigFactor = 0.5; // radians per unit ambiguity
    public static final double kMinSigmaTheta = 0.05; // radians (~2.8 degrees)
    public static final double kMaxSigmaTheta = Math.PI; // radians (180 degrees)
    public static final double kDefaultSigmaX = 0.7;
    public static final double kDefaultSigmaY = 0.7;
    public static final double kDefaultSigmaTheta = 9999999; // never trust vision for rotation by default

    // Temporal Filtering (Tier 3 Logic)
    private static final double kFarTagDistance = 4.0; // meters
    private static final int kFarTagRequiredConsecutive = 3;
    private static final double kFarTagConsecutiveWindow = 0.6; // seconds
    public static final double kFarTagSigmaFloor = 1.0; // meters
    private int m_farTagConsecutiveCount = 0;
    private double m_firstFarTagTime = 0.0;

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

    /**
     * Process vision measurements and add them to the drivetrain with appropriate gating and 
     * per-measurement standard deviations. This method implements several layers of outlier rejection 
     * to avoid bad vision measurements from negatively impacting the pose estimator, including 
     * checks on robot speed, measurement ambiguity, and distance to target. 
     * It also includes logic to require several consecutive good measurements if the 
     * detected tags are far away, as these measurements tend to be less reliable.
     */
    public void addVisionMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();
        Pose2d currentOdometryPose = drivetrain.getPose();

        LimelightHelpers.SetRobotOrientation(
                "limelight",
                driveState.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate poseEstimateMT1 = getPoseEstimateMT1();

        addVisionMeasurements(poseEstimateMT1, driveState, currentOdometryPose);
    }

    public void addVisionMeasurements(LimelightHelpers.PoseEstimate poseEstimateMT1, SwerveDriveState driveState, Pose2d currentOdometryPose) {
        // compute approximate linear speed from drivetrain reported speeds (more
        // stable)
        double linearSpeed = Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond);

        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimateMT1.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimateMT1.pose.getY());
        SmartDashboard.putNumber("Vision/avgTagDist", poseEstimateMT1.avgTagDist);
        SmartDashboard.putNumber("Vision/tagCount", poseEstimateMT1.tagCount);

        boolean doRejectUpdate = false;
        if (poseEstimateMT1.tagCount == 1 && poseEstimateMT1.rawFiducials.length == 1) {
            if (poseEstimateMT1.rawFiducials[0].ambiguity > kMaxAmbiguity) {
                doRejectUpdate = true;
            }
            if (poseEstimateMT1.rawFiducials[0].distToCamera > kMaxSingleTagDist) {
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
                    && poseEstimateMT1.avgTagDist > kMinDistForOutlierReject) {
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

            double sigmaXY = kSigmaBase + kSigmaDistFactor * Math.max(0.0, poseEstimateMT1.avgTagDist)
                    + kSigmaAmbigFactor * avgAmbiguity;
            sigmaXY = Math.min(Math.max(sigmaXY, kMinSigmaXY), kMaxSigmaXY);

            // If tags are far away, increase covariance and require consecutive good frames
            boolean isFar = poseEstimateMT1.avgTagDist >= kFarTagDistance;
            if (isFar) {
                // increase sigma to downweight far-away vision measurements
                sigmaXY = Math.max(sigmaXY, kFarTagSigmaFloor);
            }

            double sigmaTheta = kSigmaThetaBase + (kSigmaThetaAmbigFactor * avgAmbiguity);
            sigmaTheta = Math.min(Math.max(sigmaTheta, kMinSigmaTheta), kMaxSigmaTheta);

            SmartDashboard.putNumber("Vision/sigmaXY", sigmaXY);
            SmartDashboard.putNumber("Vision/sigmaTheta", sigmaTheta);

            // If far away, require several consecutive good frames within a window before
            // applying
            double now = poseEstimateMT1.timestampSeconds;
            boolean allowApply = true;
            if (isFar) {
                if (m_farTagConsecutiveCount == 0) {
                    m_firstFarTagTime = now;
                }
                m_farTagConsecutiveCount++;

                // reset if outside time window
                if (now - m_firstFarTagTime > kFarTagConsecutiveWindow) {
                    m_farTagConsecutiveCount = 1;
                    m_firstFarTagTime = now;
                }

                if (m_farTagConsecutiveCount < kFarTagRequiredConsecutive) {
                    allowApply = false;
                }
            } else {
                // reset counter when not far
                m_farTagConsecutiveCount = 0;
                m_firstFarTagTime = 0.0;
            }

            if (allowApply) {
                // Build matrix and send measurement with per-measurement std devs
                drivetrain.addVisionMeasurement(
                        poseEstimateMT1.pose,
                        poseEstimateMT1.timestampSeconds,
                        VecBuilder.fill(sigmaXY, sigmaXY, sigmaTheta));

                m_lastVisionTimestamp = poseEstimateMT1.timestampSeconds;
            } else {
                SmartDashboard.putString("Vision/lastRejectReason", "far-consecutive");
            }
        }

    }

    public void close() {
        drivetrain.close();
    }

}
