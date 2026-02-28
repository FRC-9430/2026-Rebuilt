package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

class VisionSubsystemTest {

    private CommandSwerveDrivetrain mockDrivetrain;
    private VisionSubsystem visionSubsystem;
    private SwerveDriveState mockDriveState;

    @BeforeAll
    static void initHAL() {
        // Initialize the HAL to ensure SmartDashboard and SubsystemBase work correctly
        edu.wpi.first.hal.HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        // Mock the drivetrain
        mockDrivetrain = mock(CommandSwerveDrivetrain.class);

        // Setup default drive state
        mockDriveState = new SwerveDriveState();
        mockDriveState.Pose = new Pose2d();
        mockDriveState.Speeds = new ChassisSpeeds();

        // When drivetrain.getState() is called, return our mock state object
        when(mockDrivetrain.getState()).thenReturn(mockDriveState);
        // When drivetrain.getPose() is called, return a default pose
        when(mockDrivetrain.getPose()).thenReturn(new Pose2d());

        // We pass the mock drivetrain to the constructor
        visionSubsystem = new VisionSubsystem(mockDrivetrain);
    }

    @AfterEach
    void tearDown() {
        visionSubsystem.close();
    }

    /**
     * Helper to create a PoseEstimate with specific parameters for testing.
     */
    private PoseEstimate createPoseEstimate(double timestamp, int tagCount, double avgDist, double avgAmbiguity,
            Pose2d pose) {
        RawFiducial[] fids = new RawFiducial[tagCount];
        for (int i = 0; i < tagCount; i++) {
            // RawFiducial(int id, double txnc, double tync, double ta, double distToCamera,
            // double distToRobot, double ambiguity)
            fids[i] = new RawFiducial(i + 1, 0, 0, 0, avgDist, avgDist, avgAmbiguity);
        }

        // for readability, the object's data structure:
        // PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
        // int tagCount, double tagSpan, double avgTagDist,
        // double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2)
        return new PoseEstimate(pose, timestamp, 0.0, tagCount, 0.0, avgDist, 0.0, fids, false);
    }

    /**
     * GIVEN a VisionSubsystem
     * WHEN the constructor runs
     * THEN it should set the default vision measurement standard deviations on the
     * drivetrain
     */
    @Test
    void testConstructorSetsStdDevs() {
        verify(mockDrivetrain).setVisionMeasurementStdDevs(any());
    }

    /**
     * GIVEN a valid vision pose estimate with low ambiguity and close distance
     * WHEN addVisionMeasurements() is called
     * THEN the measurement should be added to the drivetrain
     */
    @Test
    void testValidUpdate() {
        double timestamp = 10.0;
        Pose2d visionPose = new Pose2d(2.0, 2.0, new Rotation2d());
        PoseEstimate est = createPoseEstimate(timestamp, 2, 2.0, 0.1, visionPose);

        // Execute
        visionSubsystem.addVisionMeasurements(est, mockDriveState, visionPose);

        verify(mockDrivetrain).addVisionMeasurement(eq(visionPose), eq(timestamp), any());
    }

    /**
     * GIVEN a vision update with 0 tags.
     * WHEN addVisionMeasurements() is called.
     * THEN the measurement should be rejected and not added to the drivetrain.
     */
    @Test
    void testRejectNoTags() {
        PoseEstimate est = createPoseEstimate(10.0, 0, 0.0, 0.0, new Pose2d());
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d());

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN a vision update with a single tag that has high ambiguity (> 0.5)
     * WHEN addVisionMeasurements() is called
     * THEN the measurement should be rejected
     */
    @Test
    void testRejectSingleTagHighAmbiguity() {
        // Use a value slightly higher than the constant to ensure rejection
        double highAmbiguity = VisionSubsystem.kMaxAmbiguity + 0.1;
        PoseEstimate est = createPoseEstimate(10.0, 1, 2.0, highAmbiguity, new Pose2d());
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d());

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN a vision update with a single tag that is far away (> 3.0m)
     * WHEN addVisionMeasurements() is called
     * THEN the measurement should be rejected
     */
    @Test
    void testRejectSingleTagFarDistance() {
        // Use a value higher than the single tag limit
        double tooFar = VisionSubsystem.kMaxSingleTagDist + 0.5;
        PoseEstimate est = createPoseEstimate(10.0, 1, tooFar, 0.1, new Pose2d());
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d());

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN the robot is rotating at a high angular rate (> 1.5 rad/s)
     * WHEN addVisionMeasurements() is called with a valid vision pose
     * THEN the measurement should be rejected
     */
    @Test
    void testRejectHighAngularRate() {
        mockDriveState.Speeds.omegaRadiansPerSecond = 2.0; // > 1.5 threshold

        PoseEstimate est = createPoseEstimate(10.0, 2, 2.0, 0.1, new Pose2d());
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d());

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN the robot is moving at a high linear speed (> 1.0 m/s)
     * WHEN addVisionMeasurements() is called with a valid vision pose
     * THEN the measurement should be rejected
     */
    @Test
    void testRejectHighLinearSpeed() {
        mockDriveState.Speeds.vxMetersPerSecond = 1.5; // > 1.0 threshold

        PoseEstimate est = createPoseEstimate(10.0, 2, 2.0, 0.1, new Pose2d());
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d());

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN a valid vision update was just processed
     * WHEN another update arrives sooner than the minimum interval (0.08s)
     * THEN the second measurement should be rejected
     */
    @Test
    void testRejectTooFrequent() {
        Pose2d visionPose = new Pose2d(2.0, 2.0, new Rotation2d());

        // First update (valid)
        PoseEstimate est1 = createPoseEstimate(10.0, 2, 2.0, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est1, mockDriveState, visionPose);
        verify(mockDrivetrain, times(1)).addVisionMeasurement(any(), eq(10.0), any());

        // Second update (too soon, dt = 0.05 < 0.08 threshold)
        PoseEstimate est2 = createPoseEstimate(10.05, 2, 2.0, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est2, mockDriveState, visionPose);

        // Should still be 1 call total
        verify(mockDrivetrain, times(1)).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN the robot is stopped and the vision pose is significantly different (>
     * 1.0m) from odometry
     * WHEN addVisionMeasurements() is called
     * THEN the measurement should be rejected as an outlier
     */
    @Test
    void testRejectLargeJumpWhenStopped() {
        when(mockDrivetrain.getPose()).thenReturn(new Pose2d(0, 0, new Rotation2d()));

        // Vision pose far away (2m > 1m threshold)
        Pose2d visionPose = new Pose2d(2.0, 0, new Rotation2d());
        PoseEstimate est = createPoseEstimate(10.0, 2, 2.0, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est, mockDriveState, new Pose2d(0, 0, new Rotation2d()));

        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());
    }

    /**
     * GIVEN a tag is detected at a far distance (> 4.0m)
     * WHEN updates are received consecutively
     * THEN the measurement should only be applied after a specific number of
     * consecutive valid frames (3)
     */
    @Test
    void testFarTagConsecutiveLogic() {
        double dist = 5.0;
        Pose2d visionPose = new Pose2d(5.0, 0, new Rotation2d());

        // Frame 1: Should be rejected (count = 1)
        PoseEstimate est1 = createPoseEstimate(10.0, 2, dist, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est1, mockDriveState, visionPose);
        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());

        // Frame 2: Should be rejected (count = 2)
        PoseEstimate est2 = createPoseEstimate(10.1, 2, dist, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est2, mockDriveState, visionPose);
        verify(mockDrivetrain, never()).addVisionMeasurement(any(), anyDouble(), any());

        // Frame 3: Should be accepted (count = 3)
        PoseEstimate est3 = createPoseEstimate(10.2, 2, dist, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est3, mockDriveState, visionPose);
        verify(mockDrivetrain, times(1)).addVisionMeasurement(eq(visionPose), eq(10.2), any());
    }

    /**
     * GIVEN a valid vision update
     * WHEN addVisionMeasurements() is called
     * THEN the standard deviations (sigmas) passed to the drivetrain should be
     * calculated based on distance and ambiguity
     */
    @Test
    void testSigmaCalculation() {
        Pose2d visionPose = new Pose2d(2.0, 2.0, new Rotation2d());
        PoseEstimate est = createPoseEstimate(10.0, 2, 2.0, 0.1, visionPose);
        visionSubsystem.addVisionMeasurements(est, mockDriveState, visionPose);

        ArgumentCaptor<Matrix<N3, N1>> matrixCaptor = ArgumentCaptor.forClass(Matrix.class);
        verify(mockDrivetrain).addVisionMeasurement(eq(visionPose), eq(10.0), matrixCaptor.capture());

        Matrix<N3, N1> stdDevs = matrixCaptor.getValue();

        // Calculate expected sigma based on the constants in the subsystem
        double expectedSigmaXY = VisionSubsystem.kSigmaBase
                + (VisionSubsystem.kSigmaDistFactor * 2.0)
                + (VisionSubsystem.kSigmaAmbigFactor * 0.1);

        double expectedSigmaTheta = VisionSubsystem.kSigmaThetaBase
                + (VisionSubsystem.kSigmaThetaAmbigFactor * 0.1);

        assertEquals(expectedSigmaXY, stdDevs.get(0, 0), 0.001, "Sigma X incorrect");
        assertEquals(expectedSigmaXY, stdDevs.get(1, 0), 0.001, "Sigma Y incorrect");
        assertEquals(expectedSigmaTheta, stdDevs.get(2, 0), 0.001, "Sigma Theta incorrect");
    }
}
