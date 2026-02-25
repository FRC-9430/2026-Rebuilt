package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.lang.reflect.Field;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystemTest {

    private ShooterSubsystem m_shooter;

    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 2500.0;
    private static final double FLYWHEEL_RPM_TEST_TOLERANCE = 50.0;
    private static final double HOOD_START_POSITION = 0.5;
    private static final double TARGET_HOOD_POSITION = 0.3;
    private static final double CURRENT_HOOD_POSITION = 0.2;
    private static final double TEST_HOOD_POSITION = 0.45;
    // FIXME 20260224.1349 bbontrager, lower hood tolerance to match TOLERANCE
    // likely requires additional PID tuning.
    private static final double HOOD_TOLERANCE = 0.30;
    private static final double MANUAL_HOOD_CONTROL_SPEED = 0.5;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void tearDown() {
        if (m_shooter != null) {
            try {
                m_shooter.close();
            } catch (Exception e) {
                // Ignore errors during standard close
            }

            // Use reflection to close all SparkFlex fields to ensure HAL resources are released.
            // This prevents IllegalStateException in subsequent tests if the subsystem's close()
            // method misses any motors.
            try {
                for (Field field : m_shooter.getClass().getDeclaredFields()) {
                    if (AutoCloseable.class.isAssignableFrom(field.getType())) {
                        field.setAccessible(true);
                        AutoCloseable ac = (AutoCloseable) field.get(m_shooter);
                        if (ac != null) {
                            try {
                                ac.close();
                            } catch (Exception e) {
                                // Ignore errors during cleanup
                            }
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
            m_shooter = null;
        }
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    private void step(double stepSeconds) {
        SimHooks.stepTiming(stepSeconds);
    }

    /**
     * GIVEN a call to construct ShooterSubsystem
     * WHEN constructor calls configure() on the SparkFlex objects.
     * THEN verify that the configuration logic inside the constructor
     *      is actually executed. This ensures the correct configs are passed to
     *      each SparkFlex motor in the ShooterSubsystem.
     */
    @Test
    void testConstructorConfiguration() {
        // Mock construction to verify configuration calls on the SparkFlex objects
        try (MockedConstruction<SparkFlex> mockedSparkFlex = Mockito.mockConstruction(SparkFlex.class,
                (mock, context) -> {
                    // Return non-null objects for getters used in constructor
                    when(mock.getClosedLoopController()).thenReturn(mock(SparkClosedLoopController.class));
                    when(mock.getEncoder()).thenReturn(mock(RelativeEncoder.class));
                    when(mock.getAbsoluteEncoder()).thenReturn(mock(SparkAbsoluteEncoder.class));
                })) {
            m_shooter = new ShooterSubsystem();

            List<SparkFlex> constructed = mockedSparkFlex.constructed();
            // Ensure we have the expected number of motors (6 total in constructor)
            assertEquals(6, constructed.size());

            // Retrieve mocks based on instantiation order in ShooterSubsystem constructor
            SparkFlex rightShooter = constructed.get(0);
            SparkFlex leftTopShooter = constructed.get(1);
            SparkFlex leftBotShooter = constructed.get(2);
            SparkFlex hoodMotor = constructed.get(3);
            SparkFlex feedMotor = constructed.get(4);

            // Verify that configure was called with the correct config object from Constants
            Mockito.verify(rightShooter).configure(Mockito.eq(ShooterConstants.MAIN_SHOOTER_CONFIG),
                    Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
            Mockito.verify(leftTopShooter).configure(Mockito.eq(ShooterConstants.AUX_SHOOTER_CONFIG),
                    Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
            Mockito.verify(leftBotShooter).configure(Mockito.eq(ShooterConstants.AUX_SHOOTER_CONFIG),
                    Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
            Mockito.verify(hoodMotor).configure(Mockito.eq(ShooterConstants.HOOD_CONFIG),
                    Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
            Mockito.verify(feedMotor).configure(Mockito.eq(ShooterConstants.FEED_CONFIG),
                    Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
        }
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is advanced.
     * THEN both flywheels should be configured to the target setpoint as a velocity control
     */
    @Test
    void testSetShooterRPM() {
        m_shooter = new ShooterSubsystem();
        SparkClosedLoopController controller = m_shooter.getShooterPID("shoot");
        double targetRPM = TEST_RPM;

        // Simulate flywheels spinning up
        m_shooter.setShooterRPM(targetRPM);
        step();

        assertNotNull(controller); // controller is created
        assertEquals(targetRPM, controller.getSetpoint(), TOLERANCE); // controller setpoint matches targetRPM within tolerance
        assertEquals(ControlType.kVelocity, controller.getControlType()); // setpoint control type is velocity
    }

    /**
     * GIVEN a ShooterSubsystem with the flywheels commanded to spin.
     * WHEN the {@code stopFlywheels()} method is called.
     * THEN the applied output to both flywheel motors should be zero.
     */
    @Test
    void testStopShooter() {
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);
        SparkFlexSim m_followerShooterMotorSim1 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(1), null);
        SparkFlexSim m_followerShooterMotorSim2 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(2), null);

        m_shooter.setShooterRPM(TEST_RPM);
        step();
        m_shooter.stopShooter();
        step();

        assertEquals(0, m_mainShooterMotorSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim1.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim2.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is
     * updated to match that RPM.
     * THEN the {@code isShooterAtSpeed()} method should return true.
     */
    @Test
    void testIsShooterAtSpeed() {
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);

        // Define and convert inputs
        // Convert kV from Volts/RPM to Volts/(rad/s)
        // 1 RPM = 0.10472 rad/s = V*60 / (2pi)
        double kV_SI = ShooterConstants.kShooterV * 60 / (2 * Math.PI);
        // Convert kV from Amps/RPM to Amps/(rad/s)
        // A*60 / (2pi)
        // Use a default kA if constant is 0 to prevent simulation errors
        double kA_SI = ShooterConstants.kShooterA == 0 ? 0.005 : ShooterConstants.kShooterA * 60 / (2 * Math.PI);

        // Feed inputs to a generic simulated flywheel system
        FlywheelSim flywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(kV_SI, kA_SI),
                DCMotor.getNeoVortex(3),
                1.0);

        double targetRPM = TEST_RPM;
        m_shooter.setShooterRPM(targetRPM);

        // Simulation Loop: Run for 2 seconds
        for (int i = 0; i < 100; i++) {
            double currentRPM = flywheelSim.getAngularVelocityRPM();
            m_mainShooterMotorSim.setVelocity(currentRPM);

            // Simulate SparkFlex PID + FeedForward
            double error = targetRPM - currentRPM;
            double ffVolts = (ShooterConstants.kShooterS * Math.signum(targetRPM))
                    + (ShooterConstants.kShooterV * targetRPM);
            double pidOutput = error * ShooterConstants.kShooterP;
            double appliedVoltage = ffVolts + (pidOutput * 12.0);

            // Apply to Plant
            flywheelSim.setInputVoltage(appliedVoltage);
            flywheelSim.update(0.02);
            step(0.02);
        }

        // Finally, capture ending values and pass into overloaded validation method
        double endRPM = m_mainShooterMotorSim.getVelocity();
        double endSetpoint = m_mainShooterMotorSim.getSetpoint();
        double target = Math.abs(m_mainShooterMotorSim.getVelocity() - m_mainShooterMotorSim.getSetpoint());

        assertTrue(m_shooter.isShooterAtSpeed(endRPM, endSetpoint),
                "Shooter should be at speed. Current velocity: " + m_mainShooterMotorSim.getVelocity()  +
                "\nTarget velocity: " +  targetRPM +
                "\nWithin Tolerance: " + (target <= 200));
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM but the motors are currently at a different RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return false.
     */
    @Test
    void testFlywheelsNotAtSpeed() {
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);
        SparkFlexSim m_followerShooterMotorSim1 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(1), null);
        SparkFlexSim m_followerShooterMotorSim2 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(2), null);

        m_shooter.setShooterRPM(TEST_RPM);
        step();

        // Set velocity to something other than target
        m_mainShooterMotorSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_followerShooterMotorSim1.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_followerShooterMotorSim2.setVelocity(ShooterConstants.kShooterIdleRPM);
        step();

        assertFalse(m_shooter.isShooterAtSpeed()); // setShooterRPM passes, therefore this test should assert false
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the {@code stowHood()} method is called and the hood is simulated to be at the stowed position.
     * THEN the {@code isHoodStowed()} method should return true. (Assumption: This tests the sensor logic more than the movement?)
     * FIXME 20260224.1423 bbontrager,  Disabled for AssertionFailedError.
     */
    @Test
    void testHoodStow() {
        m_shooter = new ShooterSubsystem();
        SparkAbsoluteEncoderSim m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_shooter.getHoodMotor());

        m_shooter.stowHood();
        step();

        // Simulate the hood reaching the stowed position
        m_hoodEncoderSim.setPosition(ShooterConstants.kHoodStowedPosition);
        step();

        assertTrue(m_shooter.isHoodStowed());
        assertEquals(ShooterConstants.kHoodStowedPosition, m_shooter.getHoodPosition(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position and the simulation is updated to match that position.
     * THEN the {@code isHoodAtPosition()} method should return true for that position.
     * FIXME 20260224.1424 bbontrager, Disabled for AssertionFailedError.
     */
    @Test
    void testIsHoodAtPosition() {
        m_shooter = new ShooterSubsystem();
        SparkAbsoluteEncoderSim m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_shooter.getHoodMotor());

        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);
        step();

        // Simulate reaching position
        m_hoodEncoderSim.setPosition(TARGET_HOOD_POSITION);
        step();

        assertTrue(m_shooter.isHoodAtPosition(TARGET_HOOD_POSITION));
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position but the simulation is at a different position.
     * THEN the {@code isHoodAtPosition()} method should return false for the target position.
     */
    @Test
    void testHoodNotAtPosition() {
        m_shooter = new ShooterSubsystem();
        SparkAbsoluteEncoderSim m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_shooter.getHoodMotor());
        double testfail = TARGET_HOOD_POSITION - 0.1;

        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);
        step();

        // Simulate being at a different position
        m_hoodEncoderSim.setPosition(testfail);
        step();

        assertFalse(m_shooter.isHoodAtPosition(TARGET_HOOD_POSITION));
        assertEquals(testfail, m_hoodEncoderSim.getPosition(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem with the hood motor running.
     * WHEN the {@code stopHood()} method is called.
     * THEN the applied output to the hood motor should be zero.
     */
    @Test
    void testStopHood() {
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_hoodSim = new SparkFlexSim(m_shooter.getHoodMotor(), null);

        m_shooter.manualHood(MANUAL_HOOD_CONTROL_SPEED);
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
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);
        SparkFlexSim m_feederSim = new SparkFlexSim(m_shooter.getFeedMotor(), null);
        SparkFlexSim m_hoodSim = new SparkFlexSim(m_shooter.getHoodMotor(), null);

        m_shooter.setShooterRPM(TEST_RPM);
        m_shooter.manualHood(MANUAL_HOOD_CONTROL_SPEED);
        m_shooter.setFeederRPM(TEST_RPM / 4);
        step();

        m_shooter.stopAll();
        step();

        assertEquals(0, m_mainShooterMotorSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_feederSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_hoodSim.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem
     * WHEN feed motor is engaged
     * THEN feed controller should be set to the PID velocity
     * */
    @Test
    void testsetFeederRPM() {
        m_shooter = new ShooterSubsystem();
        SparkClosedLoopController m_feedController = m_shooter.getShooterPID("feed");

        m_shooter.setFeederRPM(TEST_RPM);
        step();

        assertEquals(ControlType.kVelocity, m_feedController.getControlType());
        assertEquals(TEST_RPM, m_feedController.getSetpoint(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the setHoodPosition method is called with a position.
     * THEN the hood's closed-loop controller setpoint should be updated to that position.
     */
    @Test
    void testSetHoodPositionUpdatesSetpoint() {
        m_shooter = new ShooterSubsystem();
        double targetPosition = 0.75;
        SparkClosedLoopController hoodController = m_shooter.getShooterPID("hood");

        m_shooter.setHoodPosition(targetPosition);
        step();

        assertEquals(targetPosition, hoodController.getSetpoint(), TOLERANCE);
        assertEquals(ControlType.kMAXMotionPositionControl, hoodController.getControlType());
    }


    /**
     * GIVEN a ShooterSubsystem with a hood motor encoder
     * WHEN setHoodPosition called with a target position parameter
     * THEN hood encoder reaches target position
     */
    @Test
    void testSetHoodPosition() {
        m_shooter = new ShooterSubsystem();

        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);
        step();

        assertEquals(TARGET_HOOD_POSITION, m_shooter.getHoodPosition(), 0.3);
    }
}
