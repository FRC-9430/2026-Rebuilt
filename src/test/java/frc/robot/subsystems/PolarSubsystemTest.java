package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

class PolarSubsystemTest {

    private CommandSwerveDrivetrain mockDrivetrain;
    private PolarSubsystem polarSubsystem;
    private SwerveDriveState mockDriveState;

    private final Translation2d BLUE_HUB = new Translation2d(4.6525, 4.034);
    private final double MAX_SPEED = 4.0;
    private final double MAX_ANGULAR_RATE = Math.PI;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        mockDrivetrain = mock(CommandSwerveDrivetrain.class);
        mockDriveState = new SwerveDriveState();
        mockDriveState.Pose = new Pose2d();
        mockDriveState.Speeds = new ChassisSpeeds();

        when(mockDrivetrain.getState()).thenReturn(mockDriveState);
        when(mockDrivetrain.getPose()).thenReturn(new Pose2d());

        try (MockedStatic<DriverStation> ds = mockStatic(DriverStation.class)) {
            ds.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));
            polarSubsystem = new PolarSubsystem(mockDrivetrain);
        }
    }

    /**
     * GIVEN the robot is in targeting mode
     * WHEN a forward driver input is provided
     * THEN the robot should translate radially toward the hub center
     */
    @Test
    void testRadialMovementForward() {
        // Robot at (10, 4.034), Hub at (4.65, 4.034). Forward is -X.
        Pose2d robotPose = new Pose2d(10.0, 4.034, Rotation2d.k180deg);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);

        // radialIn = -1.0 (Forward stick)
        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, -1.0, 0.0, MAX_SPEED, MAX_ANGULAR_RATE, false);

        // Should move in +X robot-relative direction (forward toward hub)
        assertTrue(speeds.vxMetersPerSecond > 0, "Robot should move toward hub center");
        assertEquals(0.0, speeds.vyMetersPerSecond, 0.001, "Robot should have no lateral movement");
    }

    /**
     * GIVEN the robot is in targeting mode
     * WHEN a lateral driver input is provided
     * THEN the robot should translate tangentially (circularly) about the hub center
     */
    @Test
    void testOrbitalMovement() {
        // Robot at (10, 4.034), Hub at (4.65, 4.034).
        Pose2d robotPose = new Pose2d(10.0, 4.034, Rotation2d.k180deg);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);

        // orbitalIn = 1.0 (Right stick)
        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, 0.0, 1.0, MAX_SPEED, MAX_ANGULAR_RATE, false);

        // At this position, tangential is +/- Y.
        assertEquals(0.0, speeds.vxMetersPerSecond, 0.001, "Robot should have no radial movement");
        assertNotEquals(0.0, speeds.vyMetersPerSecond, "Robot should move tangentially");

        // Verify circular nature: velocity vector must be perpendicular to radial vector
        Translation2d radialVec = BLUE_HUB.minus(robotPose.getTranslation());
        Translation2d velocityVec = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        // Dot product of perpendicular vectors is 0
        double dotProduct = radialVec.getX() * velocityVec.getX() + radialVec.getY() * velocityVec.getY();
        assertEquals(0.0, dotProduct, 0.001, "Movement must be circular (perpendicular to radius)");
    }

    /**
     * GIVEN the robot is in targeting mode
     * WHEN getPolarDriveSpeeds is called
     * THEN the rotation (omega) should be calculated by the subsystem to face the hub,
     *      ignoring any manual driver rotation input.
     */
    @Test
    void testRotationControlledBySubsystem() {
        // Robot at (10, 10), Hub at (4.65, 4.034). Robot facing 0deg (away).
        Pose2d robotPose = new Pose2d(10.0, 10.0, Rotation2d.kZero);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);

        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, 0.0, 0.0, MAX_SPEED, MAX_ANGULAR_RATE, false);

        // Subsystem should produce omega to turn robot toward hub
        assertNotEquals(0.0, speeds.omegaRadiansPerSecond, "Subsystem must control rotation to aim at hub");
    }

    /**
     * GIVEN the robot is stationary relative to the hub (zero tangential velocity)
     * WHEN lead shot compensation is enabled
     * THEN the drivetrain heading should maintain a 0 degree offset from the hub center
     */
    @Test
    void testHeadingNoLeadWhenStationary() {
        // Robot at (10, 4.034), Hub at (4.65, 4.034). Facing 180 (direct).
        Pose2d robotPose = new Pose2d(10.0, 4.034, Rotation2d.k180deg);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);
        mockDriveState.Speeds = new ChassisSpeeds(0, 0, 0);

        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, 0.0, 0.0, MAX_SPEED, MAX_ANGULAR_RATE, true);

        // With 0 tangential velocity, omega should be 0 as we are already facing the target
        assertEquals(0.0, speeds.omegaRadiansPerSecond, 0.001, "Heading offset must be 0 when tangential velocity is 0");
    }

    /**
     * GIVEN the robot is orbiting Clockwise (Tangentially Right)
     * WHEN lead shot compensation is enabled
     * THEN the drivetrain must maintain a positive angular offset (aiming left of center)
     */
    @Test
    void testHeadingLeadClockwiseOrbit() {
        // Robot at (10, 4.034), Hub at (4.65, 4.034). Facing 180.
        Pose2d robotPose = new Pose2d(10.0, 4.034, Rotation2d.k180deg);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);

        // Mock a Clockwise (Right) tangential velocity
        // In this setup, moving toward -Y is Clockwise.
        mockDriveState.Speeds = new ChassisSpeeds(0, -2.0, 0);

        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, 0.0, 1.0, MAX_SPEED, MAX_ANGULAR_RATE, true);

        // To aim "Left" of center while moving "Right", the robot needs a positive angular correction
        // relative to the direct heading.
        assertTrue(speeds.omegaRadiansPerSecond > 0, "CW orbit must result in positive angular offset (aiming left)");
    }

    /**
     * GIVEN the robot is orbiting Counter-Clockwise (Tangentially Left)
     * WHEN lead shot compensation is enabled
     * THEN the drivetrain must maintain a negative angular offset (aiming right of center)
     */
    @Test
    void testHeadingLeadCounterClockwiseOrbit() {
        // Robot at (10, 4.034), Hub at (4.65, 4.034). Facing 180.
        Pose2d robotPose = new Pose2d(10.0, 4.034, Rotation2d.k180deg);
        when(mockDrivetrain.getPose()).thenReturn(robotPose);

        // Mock a Counter-Clockwise (Left) tangential velocity (+Y)
        mockDriveState.Speeds = new ChassisSpeeds(0, 2.0, 0);

        ChassisSpeeds speeds = polarSubsystem.getPolarDriveSpeeds(robotPose, 0.0, -1.0, MAX_SPEED, MAX_ANGULAR_RATE, true);

        // To aim "Right" of center while moving "Left", the robot needs a negative angular correction
        assertTrue(speeds.omegaRadiansPerSecond < 0, "CCW orbit must result in negative angular offset (aiming right)");
    }

    /**
     * GIVEN a RobotContainer
     * WHEN the left bumper is pressed
     * THEN the drive mode should toggle between CARTESIAN and POLAR
     */
    @Test
    void testDriveModeToggle() {
        // This logic is in RobotContainer. We verify the state management.
        RobotContainer container;
        try (MockedStatic<DriverStation> ds = mockStatic(DriverStation.class)) {
            ds.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));
            container = new RobotContainer();
        }

        // Initial state
        assertTrue(container.isCartesian());
        assertFalse(container.isPolar());

        // Toggle to Polar
        container.setPolar();
        assertTrue(container.isPolar());
        assertFalse(container.isCartesian());

        // Toggle back to Cartesian
        container.setCartesian();
        assertTrue(container.isCartesian());
    }
}
