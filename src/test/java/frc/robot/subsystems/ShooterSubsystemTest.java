package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.ShooterConstants;

// TODO: The tests in this file are running against a largely unimplemented
// frc.robot.subsystems.ShooterSubsystem, and will need to be updated as the
// features of the subsystem are implemented.
public class ShooterSubsystemTest {

    private ShooterSubsystem m_shooter;
    private SparkFlexSim m_topFlywheelSim;
    private SparkFlexSim m_bottomFlywheelSim;
    private SparkFlexSim m_hoodSim;
    private SparkAbsoluteEncoderSim m_hoodEncoderSim;

    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 2500.0;
    private static final double FLYWHEEL_RPM_TEST_TOLERANCE = 50.0;
    private static final double HOOD_START_POSITION = 0.5;
    private static final double TARGET_HOOD_POSITION = 0.3;
    private static final double CURRENT_HOOD_POSITION = 0.2;
    private static final double TEST_HOOD_POSITION = 0.45;
    private static final double MANUAL_HOOD_CONTROL_SPEED = 0.5;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        m_shooter = new ShooterSubsystem();
        m_topFlywheelSim = new SparkFlexSim(m_shooter.getTopFlywheelMotor(), null);
        m_bottomFlywheelSim = new SparkFlexSim(m_shooter.getBottomFlywheelMotor(), null);
        m_hoodSim = new SparkFlexSim(m_shooter.getHoodMotor(), null);
        m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_shooter.getHoodMotor());
    }

    @AfterEach
    void tearDown() {
        m_shooter.stopAll();
        m_shooter = null;
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    /**
     * GIVEN a new ShooterSubsystem is created.
     * WHEN the subsystem is initialized in {@code setUp()}.
     * THEN all motor controller objects should be instantiated and not null.
     */
    @Test
    void testMotorsInitialized() {
        assertNotNull(m_shooter.getTopFlywheelMotor());
        assertNotNull(m_shooter.getBottomFlywheelMotor());
        assertNotNull(m_shooter.getHoodMotor());
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is advanced.
     * THEN both flywheels should be simulated to be spinning at the same velocity and in the same direction.
     */
    @Test
    void testFlywheelsSpinUp() {
        double targetRPM = ShooterConstants.kShooterTargetRPM;
        m_shooter.setFlywheelSpeeds(targetRPM, targetRPM);
        step();

        // Simulate flywheels spinning up
        m_topFlywheelSim.setVelocity(targetRPM);
        m_bottomFlywheelSim.setVelocity(targetRPM);
        step();

        double topVelocity = m_topFlywheelSim.getVelocity();
        double bottomVelocity = m_bottomFlywheelSim.getVelocity();

        assertEquals(topVelocity, bottomVelocity, TOLERANCE);
        assertEquals(Math.signum(topVelocity), Math.signum(bottomVelocity), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem with the flywheels commanded to spin.
     * WHEN the {@code stopFlywheels()} method is called.
     * THEN the applied output to both flywheel motors should be zero.
     */
    @Test
    void testFlywheelsStop() {
        m_shooter.setFlywheelSpeeds(ShooterConstants.kShooterTargetRPM, ShooterConstants.kShooterTargetRPM);
        step();

        m_shooter.stopFlywheels();
        step();

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is updated to match that RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return true.
     */
    @Test
    void testFlywheelsAtSpeed() {
        double targetRPM = ShooterConstants.kShooterTargetRPM;
        m_shooter.setFlywheelSpeeds(targetRPM, targetRPM);

        m_topFlywheelSim.setVelocity(targetRPM);
        m_bottomFlywheelSim.setVelocity(targetRPM);
        step();

        assertTrue(m_shooter.flywheelsAtSpeed());
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM but the simulation is at a different RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return false.
     */
    @Test
    void testFlywheelsNotAtSpeed() {
        m_shooter.setFlywheelSpeeds(ShooterConstants.kShooterTargetRPM, ShooterConstants.kShooterTargetRPM);

        m_topFlywheelSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_bottomFlywheelSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        step();

        assertFalse(m_shooter.flywheelsAtSpeed());
    }

    /**
     * GIVEN a ShooterSubsystem with flywheels simulated to be running at a known RPM.
     * WHEN {@code getFlywheelRPM()} is called.
     * THEN the method should return the average RPM of the flywheels within a tolerance.
     */
    @Test
    void testGetFlywheelRPM() {
        m_topFlywheelSim.setVelocity(TEST_RPM);
        m_bottomFlywheelSim.setVelocity(TEST_RPM);
        step();

        assertEquals(TEST_RPM, m_shooter.getFlywheelRPM(), FLYWHEEL_RPM_TEST_TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the {@code stowHood()} method is called and the hood is simulated to be at the stowed position.
     * THEN the {@code isHoodStowed()} method should return true. (Assumption: This tests the sensor logic more than the movement?)
     */
    @Test
    void testHoodStow() {
        m_hoodEncoderSim.setPosition(HOOD_START_POSITION);
        step();

        m_shooter.stowHood();
        m_hoodEncoderSim.setPosition(ShooterConstants.kHoodStowedPosition);
        step();

        assertTrue(m_shooter.isHoodStowed());
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position and the simulation is updated to match that position.
     * THEN the {@code hoodAtPosition()} method should return true for that position.
     */
    @Test
    void testHoodAtPosition() {
        m_shooter.setShootingAngle(TARGET_HOOD_POSITION);
        m_hoodEncoderSim.setPosition(TARGET_HOOD_POSITION);
        step();

        assertTrue(m_shooter.hoodAtPosition(TARGET_HOOD_POSITION));
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position but the simulation is at a different position.
     * THEN the {@code hoodAtPosition()} method should return false for the target position.
     */
    @Test
    void testHoodNotAtPosition() {
        m_shooter.setShootingAngle(TARGET_HOOD_POSITION);
        m_hoodEncoderSim.setPosition(CURRENT_HOOD_POSITION);
        step();

        assertFalse(m_shooter.hoodAtPosition(TARGET_HOOD_POSITION));
    }

    /**
     * GIVEN a ShooterSubsystem with the hood simulated to be at a known position.
     * WHEN {@code getHoodPosition()} is called.
     * THEN the method should return the known position of the hood.
     */
    @Test
    void testGetHoodPosition() {
        m_hoodEncoderSim.setPosition(TEST_HOOD_POSITION);
        step();

        assertEquals(TEST_HOOD_POSITION, m_shooter.getHoodPosition(), 0.01);
    }

    /**
     * GIVEN a ShooterSubsystem with the hood motor running.
     * WHEN the {@code stopHood()} method is called.
     * THEN the applied output to the hood motor should be zero.
     */
    @Test
    void testStopHood() {
        m_shooter.manualHoodControl(MANUAL_HOOD_CONTROL_SPEED);
        step();

        m_shooter.stopHood();
        step();

        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem with all motors running.
     * WHEN the {@code stopAll()} method is called.
     * THEN the applied output to all motors in the subsystem should be zero.
     */
    @Test
    void testStopAll() {
        m_shooter.setFlywheelSpeeds(ShooterConstants.kShooterTargetRPM, ShooterConstants.kShooterTargetRPM);
        m_shooter.manualHoodControl(MANUAL_HOOD_CONTROL_SPEED);
        step();

        m_shooter.stopAll();
        step();

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }

     // complex shooter calculation tests assert true when motion matches expected rotational offset
     // NOTE: passing these tests does NOT imply accurate shots; just that the core functionality works

    /**
     * GIVEN a ShooterSubsystem with fuel, ready to shoot.
     * WHEN the robot's tangential velocity relative to the hub is zero (stationary or moving radially).
     * THEN the drivetrain heading must maintain a 0deg offset from the center of the hub.
     */
    @Test
    void testAimingZeroTangentialVelocity() {
        Translation2d robotPose = new Translation2d(10, 0);
        Translation2d targetPose = new Translation2d(0, 0);
        double projectileSpeed = 10.0;

        // Case 1: Stationary
        ChassisSpeeds stationary = new ChassisSpeeds(0, 0, 0);
        Rotation2d headingStationary = m_shooter.calculateAimingHeading(robotPose, stationary, targetPose, projectileSpeed);
        // Expected: Pointing from (10,0) to (0,0) is 180 degrees (PI radians)
        assertEquals(0.0, headingStationary.minus(new Rotation2d(Math.PI)).getRadians(), TOLERANCE);

        // Case 2: Moving Radially (towards hub)
        ChassisSpeeds radialMove = new ChassisSpeeds(-2.0, 0, 0);
        Rotation2d headingRadial = m_shooter.calculateAimingHeading(robotPose, radialMove, targetPose, projectileSpeed);
        assertEquals(0.0, headingRadial.minus(new Rotation2d(Math.PI)).getRadians(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem with fuel, ready to shoot.
     * WHEN moving tangentially right (Clockwise orbit).
     * THEN the drivetrain must maintain a calculated positive angular offset (aiming left of center).
     */
    @Test
    void testAimingTangentialRightCW() {
        Translation2d robotPose = new Translation2d(10, 0);
        Translation2d targetPose = new Translation2d(0, 0);
        double projectileSpeed = 10.0;

        // Moving -Y is Right relative to facing -X (the target).
        // This corresponds to a Clockwise orbit around the hub.
        ChassisSpeeds cwVelocity = new ChassisSpeeds(0, -2.0, 0);

        Rotation2d heading = m_shooter.calculateAimingHeading(robotPose, cwVelocity, targetPose, projectileSpeed);
        Rotation2d directAngle = new Rotation2d(Math.PI);

        // Offset = Heading - DirectAngle. Expect Positive (Aim Left).
        assertTrue(heading.minus(directAngle).getRadians() > TOLERANCE, "Should have positive offset for CW motion (aim to the left of the hub)");
    }

    /**
     * GIVEN a ShooterSubsystem with fuel, ready to shoot.
     * WHEN moving Tangentially Left (Counter-clockwise orbit).
     * THEN the drivetrain must maintain a calculated negative angular offset (aiming right of center).
     */
    @Test
    void testAimingTangentialLeftCCW() {
        Translation2d robotPose = new Translation2d(10, 0);
        Translation2d targetPose = new Translation2d(0, 0);
        double projectileSpeed = 10.0;

        // Moving +Y is Left relative to facing -X (the target).
        ChassisSpeeds ccwVelocity = new ChassisSpeeds(0, 2.0, 0);

        Rotation2d heading = m_shooter.calculateAimingHeading(robotPose, ccwVelocity, targetPose, projectileSpeed);
        Rotation2d directAngle = new Rotation2d(Math.PI);

        // Offset = Heading - DirectAngle. Expect Negative (Aim Right).
        assertTrue(heading.minus(directAngle).getRadians() < -TOLERANCE, "Should have negative offset for CCW motion (aim to the right of hub)");
    }
}
