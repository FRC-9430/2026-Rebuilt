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

public class ShooterSubsystemTest {

    private ShooterSubsystem m_shooter;
    private SparkFlexSim m_topFlywheelSim;
    private SparkFlexSim m_bottomFlywheelSim;
    private SparkFlexSim m_hoodSim;
    private SparkAbsoluteEncoderSim m_hoodEncoderSim;

    private static final double TOLERANCE = 0.01;

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

    @Test
    void testMotorsInitialized() {
        assertNotNull(m_shooter.getTopFlywheelMotor());
        assertNotNull(m_shooter.getBottomFlywheelMotor());
        assertNotNull(m_shooter.getHoodMotor());
    }

    @Test
    void testAllFlywheelsSpinTogether() {
        m_shooter.runFlywheels();
        SimHooks.stepTiming(0.02);

        double topOutput = m_topFlywheelSim.getAppliedOutput();
        double bottomOutput = m_bottomFlywheelSim.getAppliedOutput();

        assertEquals(topOutput, bottomOutput, TOLERANCE);
    }

    @Test
    void testFlywheelsSameDirection() {
        m_topFlywheelSim.setVelocity(1000);
        m_bottomFlywheelSim.setVelocity(1000);

        m_shooter.setFlywheelRPM(ShooterConstants.kShooterTargetRPM);
        SimHooks.stepTiming(0.02);

        double topOutput = m_topFlywheelSim.getAppliedOutput();
        double bottomOutput = m_bottomFlywheelSim.getAppliedOutput();

        if (topOutput != 0) {
            assertTrue(Math.signum(topOutput) == Math.signum(bottomOutput) || bottomOutput == 0);
        }
    }

    @Test
    void testFlywheelsStop() {
        m_shooter.runFlywheels();
        SimHooks.stepTiming(0.02);

        m_shooter.stopFlywheels();
        SimHooks.stepTiming(0.02);

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
    }

    @Test
    void testFlywheelsAtSpeed() {
        double targetRPM = ShooterConstants.kShooterTargetRPM;
        m_shooter.setFlywheelRPM(targetRPM);

        m_topFlywheelSim.setVelocity(targetRPM);
        m_bottomFlywheelSim.setVelocity(targetRPM);
        SimHooks.stepTiming(0.02);

        assertTrue(m_shooter.flywheelsAtSpeed());
    }

    @Test
    void testFlywheelsNotAtSpeed() {
        m_shooter.setFlywheelRPM(ShooterConstants.kShooterTargetRPM);

        m_topFlywheelSim.setVelocity(100);
        m_bottomFlywheelSim.setVelocity(100);
        SimHooks.stepTiming(0.02);

        assertFalse(m_shooter.flywheelsAtSpeed());
    }

    @Test
    void testGetFlywheelRPM() {
        double testRPM = 2500.0;
        m_topFlywheelSim.setVelocity(testRPM);
        m_bottomFlywheelSim.setVelocity(testRPM);
        SimHooks.stepTiming(0.02);

        assertEquals(testRPM, m_shooter.getFlywheelRPM(), 50.0);
    }

    @Test
    void testHoodStow() {
        m_hoodEncoderSim.setPosition(0.5);
        SimHooks.stepTiming(0.02);

        m_shooter.stowHood();
        m_hoodEncoderSim.setPosition(ShooterConstants.kHoodStowedPosition);
        SimHooks.stepTiming(0.02);

        assertTrue(m_shooter.isHoodStowed());
    }

    @Test
    void testHoodAtPosition() {
        double targetPos = 0.3;
        m_shooter.setHoodPosition(targetPos);
        m_hoodEncoderSim.setPosition(targetPos);
        SimHooks.stepTiming(0.02);

        assertTrue(m_shooter.hoodAtPosition());
    }

    @Test
    void testHoodNotAtPosition() {
        m_shooter.setHoodPosition(0.8);
        m_hoodEncoderSim.setPosition(0.2);
        SimHooks.stepTiming(0.02);

        assertFalse(m_shooter.hoodAtPosition());
    }

    @Test
    void testGetHoodPosition() {
        double testPosition = 0.45;
        m_hoodEncoderSim.setPosition(testPosition);
        SimHooks.stepTiming(0.02);

        assertEquals(testPosition, m_shooter.getHoodPosition(), 0.01);
    }

    @Test
    void testStopHood() {
        m_shooter.manualHoodControl(0.5);
        SimHooks.stepTiming(0.02);

        m_shooter.stopHood();
        SimHooks.stepTiming(0.02);

        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }

    @Test
    void testStopAll() {
        m_shooter.runFlywheels();
        m_shooter.manualHoodControl(0.5);
        SimHooks.stepTiming(0.02);

        m_shooter.stopAll();
        SimHooks.stepTiming(0.02);

        assertEquals(0, m_topFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_bottomFlywheelSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }
}