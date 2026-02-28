package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

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
     * Process vision measurements and add them to the drivetrain with appropriate
     * gating and
     * per-measurement standard deviations. This method implements several layers of
     * outlier rejection
     * to avoid bad vision measurements from negatively impacting the pose
     * estimator, including
     * checks on robot speed, measurement ambiguity, and distance to target.
     * It also includes logic to require several consecutive good measurements if
     * the
     * detected tags are far away, as these measurements tend to be less reliable.
     */
    public void addVisionMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();
        Pose2d odometryPose = drivetrain.getPose();

        LimelightHelpers.SetRobotOrientation(
                "limelight",
                driveState.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate = getPoseEstimateMT1();

        addVisionMeasurements(estimate, driveState, odometryPose);
    }

    public void addVisionMeasurements(LimelightHelpers.PoseEstimate estimate, SwerveDriveState driveState,
            Pose2d odometryPose) {

        double linearSpeed = Math.hypot(
                driveState.Speeds.vxMetersPerSecond,
                driveState.Speeds.vyMetersPerSecond);

        LimelightHelpers.SetRobotOrientation(
                "limelight",
                driveState.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0);

        logVisionData(estimate);

        if (shouldRejectMeasurement(estimate, driveState, linearSpeed))
            return;

        double distanceToOdo = estimate.pose.getTranslation()
                .getDistance(odometryPose.getTranslation());

        if (isStoppedOutlier(estimate, linearSpeed, distanceToOdo))
            return;

        double avgAmbiguity = getAverageAmbiguity(estimate);
        double sigmaXY = computeSigmaXY(estimate, avgAmbiguity);
        double sigmaTheta = computeSigmaTheta(avgAmbiguity);

        SmartDashboard.putNumber("Vision/sigmaXY", sigmaXY);
        SmartDashboard.putNumber("Vision/sigmaTheta", sigmaTheta);

        if (!allowFarTagApplication(estimate))
            return;

        drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(sigmaXY, sigmaXY, sigmaTheta));

        m_lastVisionTimestamp = estimate.timestampSeconds;
    }

    private boolean shouldRejectMeasurement(
            LimelightHelpers.PoseEstimate est,
            SwerveDriveState driveState,
            double linearSpeed) {

        if (est.tagCount == 0)
            return true;

        if (est.tagCount == 1 && est.rawFiducials.length == 1) {
            var f = est.rawFiducials[0];
            if (f.ambiguity > 0.5 || f.distToCamera > 4.0)
                return true;
        }

        if (driveState.Speeds.omegaRadiansPerSecond > kMaxAngularRateReject)
            return true;

        if (linearSpeed > kMaxLinearSpeedReject)
            return true;

        if (Math.abs(est.timestampSeconds - m_lastVisionTimestamp) < kMinVisionInterval)
            return true;

        return false;
    }

    private boolean isStoppedOutlier(
            LimelightHelpers.PoseEstimate est,
            double linearSpeed,
            double distanceToOdo) {

        SmartDashboard.putNumber("Vision/distanceToOdometry", distanceToOdo);

        return linearSpeed < kMaxLinearSpeedReject
                && distanceToOdo > kMaxOutlierDistWhenStopped
                && est.avgTagDist > 1.0;
    }

    private double getAverageAmbiguity(LimelightHelpers.PoseEstimate est) {
        if (est.rawFiducials.length == 0)
            return 0.0;

        double total = 0;
        for (var f : est.rawFiducials)
            total += f.ambiguity;

        return total / est.rawFiducials.length;
    }

    private double computeSigmaXY(
            LimelightHelpers.PoseEstimate est,
            double avgAmbiguity) {

        double sigma = 0.06
                + 0.08 * Math.max(0.0, est.avgTagDist)
                + 0.4 * avgAmbiguity;

        sigma = MathUtil.clamp(sigma, 0.03, 2.0);

        if (est.avgTagDist >= kFarTagDistance)
            sigma = Math.max(sigma, 1.0);

        return sigma;
    }

    private double computeSigmaTheta(double avgAmbiguity) {
        double sigma = 0.5 * (0.5 + avgAmbiguity);
        return MathUtil.clamp(sigma, 0.05, Math.PI);
    }

    private boolean allowFarTagApplication(LimelightHelpers.PoseEstimate est) {
        if (est.avgTagDist < kFarTagDistance) {
            m_farTagConsecutiveCount = 0;
            m_firstFarTagTime = 0.0;
            return true;
        }

        double now = est.timestampSeconds;

        if (m_farTagConsecutiveCount == 0)
            m_firstFarTagTime = now;

        m_farTagConsecutiveCount++;

        if (now - m_firstFarTagTime > kFarTagConsecutiveWindow) {
            m_farTagConsecutiveCount = 1;
            m_firstFarTagTime = now;
        }

        if (m_farTagConsecutiveCount < kFarTagRequiredConsecutive) {
            SmartDashboard.putString("Vision/lastRejectReason", "far-consecutive");
            return false;
        }

        return true;
    }

    private void logVisionData(LimelightHelpers.PoseEstimate est) {
        SmartDashboard.putNumber("Robot Pose Cam Est X", est.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", est.pose.getY());
        SmartDashboard.putNumber("Vision/avgTagDist", est.avgTagDist);
        SmartDashboard.putNumber("Vision/tagCount", est.tagCount);
    }

    public void close() {
        drivetrain.close();
    }
}
