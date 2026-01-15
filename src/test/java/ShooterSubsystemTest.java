package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;

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
        assert HAL.initialize(500, 0);
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

    @Test
    void testMotorsInitialized() {
        assertNotNull(m_shooter.getTopFlywheelMotor());
        assertNotNull(m_shooter.getBottomFlywheelMotor());
        assertNotNull(m_shooter.getHoodMotor());
    }

    @Test
    void testFlywheelsSpinUp() {
        double targetRPM = ShooterConstants.kShooterTargetRPM;
        m_shooter.setFlywheelRPM(targetRPM);
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

    @Test
    void testFlywheelsStop() {
        m_shooter.setFlywheelRPM(ShooterConstants.kShooterTargetRPM);
        step();

        m_shooter.stopFlywheels();
        step();

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
    }

    @Test
    void testFlywheelsAtSpeed() {
        double targetRPM = ShooterConstants.kShooterTargetRPM;
        m_shooter.setFlywheelRPM(targetRPM);

        m_topFlywheelSim.setVelocity(targetRPM);
        m_bottomFlywheelSim.setVelocity(targetRPM);
        step();

        assertTrue(m_shooter.flywheelsAtSpeed());
    }

    @Test
    void testFlywheelsNotAtSpeed() {
        m_shooter.setFlywheelRPM(ShooterConstants.kShooterTargetRPM);

        m_topFlywheelSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_bottomFlywheelSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        step();

        assertFalse(m_shooter.flywheelsAtSpeed());
    }

    @Test
    void testGetFlywheelRPM() {
        m_topFlywheelSim.setVelocity(TEST_RPM);
        m_bottomFlywheelSim.setVelocity(TEST_RPM);
        step();

        assertEquals(TEST_RPM, m_shooter.getFlywheelRPM(), FLYWHEEL_RPM_TEST_TOLERANCE);
    }

    @Test
    void testHoodStow() {
        m_hoodEncoderSim.setPosition(HOOD_START_POSITION);
        step();

        m_shooter.stowHood();
        m_hoodEncoderSim.setPosition(ShooterConstants.kHoodStowedPosition);
        step();

        assertTrue(m_shooter.isHoodStowed());
    }

    @Test
    void testHoodAtPosition() {
        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);
        m_hoodEncoderSim.setPosition(TARGET_HOOD_POSITION);
        step();

        assertTrue(m_shooter.hoodAtPosition(TARGET_HOOD_POSITION));
    }

    @Test
    void testHoodNotAtPosition() {
        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);
        m_hoodEncoderSim.setPosition(CURRENT_HOOD_POSITION);
        step();

        assertFalse(m_shooter.hoodAtPosition(TARGET_HOOD_POSITION));
    }

    @Test
    void testGetHoodPosition() {
        m_hoodEncoderSim.setPosition(TEST_HOOD_POSITION);
        step();

        assertEquals(TEST_HOOD_POSITION, m_shooter.getHoodPosition(), 0.01);
    }

    @Test
    void testStopHood() {
        m_shooter.manualHoodControl(MANUAL_HOOD_CONTROL_SPEED);
        step();

        m_shooter.stopHood();
        step();

        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }

    @Test
    void testStopAll() {
        m_shooter.setFlywheelRPM(ShooterConstants.kShooterTargetRPM);
        m_shooter.manualHoodControl(MANUAL_HOOD_CONTROL_SPEED);
        step();

        m_shooter.stopAll();
        step();

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }
}
