package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import org.mockito.Mockito;
import org.mockito.MockedConstruction;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.ShooterConstants;
import net.bytebuddy.matcher.SubTypeMatcher;

// TODO: The tests in this file are running against a largely unimplemented
// frc.robot.subsystems.ShooterSubsystem, and will need to be updated as the
// features of the subsystem are implemented.
public class ShooterSubsystemTest {

    private ShooterSubsystem m_shooter;

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
    }

    @AfterEach
    void tearDown() {
        if(m_shooter != null) {
            m_shooter.close();
            m_shooter = null;
        }
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    private void step(double stepSeconds) {
        SimHooks.stepTiming(0.02);
    }

    /**
     * GIVEN a new ShooterSubsystem is created.
     * WHEN the subsystem is initialized in {@code setUp()}.
     * THEN all motor controller objects should be instantiated and not null.
     */
    @Test
    void testMotorsInitialized() {
        m_shooter = new ShooterSubsystem();

        assertNotNull(m_shooter.getMainShooterMotor());
        assertNotNull(m_shooter.getFollowerShooterMotor(1));
        assertNotNull(m_shooter.getFollowerShooterMotor(2));
        assertNotNull(m_shooter.getFeedMotor());
        assertNotNull(m_shooter.getHoodMotor());
    }

    /**
     * GIVEN a shooterSubsystem
     * WHEN the subsystem is initialized
     * THEN all PID controllers should be instantiated and not null
     */
    @Test
    void testPIDControllersInitialized() {
        m_shooter = new ShooterSubsystem();

        assertNotNull(m_shooter.getShooterPID("shoot"));
        assertNotNull(m_shooter.getShooterPID("hood"));
        assertNotNull(m_shooter.getShooterPID("feed"));
    }

    /**
     * GIVEN a call to construct ShooterSubsystem
     * WHEN constructor calls configure() on the SparkFlex objects.
     * THEN verify that the configuration logic inside the constructor is actually executed.
     */
    @Test
    void testConstructorConfiguration() {
        // Mock the construction of SparkFlex to verify configuration calls
        try (MockedConstruction<SparkFlex> mockedSparkFlex = Mockito.mockConstruction(SparkFlex.class,
                (mock, context) -> {
                    // Stub methods called in constructor to point to instances of objects the
                    // methods expect (ie. an encoder, an abs encoder, and a closedloopcontroller)
                    when(mock.getEncoder()).thenReturn(mock(RelativeEncoder.class));
                    when(mock.getAbsoluteEncoder()).thenReturn(mock(SparkAbsoluteEncoder.class));
                    when(mock.getClosedLoopController()).thenReturn(mock(SparkClosedLoopController.class));
                })) {

            m_shooter = new ShooterSubsystem();

            // Verify that 6 motors were created (Main shooter, two follower shooters, Hood, Feeder, Conveyor)
            assertEquals(6, mockedSparkFlex.constructed().size(), "Subsystem constructs 6 SparkFlex motors");

            // Verify that configure was called on each motor instance
            for (SparkFlex motor : mockedSparkFlex.constructed()) {
                verify(motor, atLeastOnce()).configure(any(), any(ResetMode.class), any(PersistMode.class));
                // TODO 20260224.1146 bbontrager, does not assert configuration was successful using REVLibError kOk.
            }

            // Close the subsystem to release resources
            m_shooter.close();
            m_shooter = null;
        }
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is advanced.
     * THEN both flywheels should be simulated to be spinning at the same velocity and in the same direction.
     */
    @Test
    void testFlywheelsSpinUp() {
        m_shooter = new ShooterSubsystem();
        double targetRPM = TEST_RPM;
        m_shooter.setShooterRPM(targetRPM);

        // Simulate flywheels spinning up
        SparkClosedLoopController controller = m_shooter.getShooterPID("shoot");

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
    void testFlywheelsStop() {
        m_shooter = new ShooterSubsystem();
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);
        SparkFlexSim m_followerShooterMotorSim1 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(1), null);
        SparkFlexSim m_followerShooterMotorSim2 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(2), null);

        m_shooter.setShooterRPM(TEST_RPM);
        m_shooter.stopShooter();

        assertEquals(0, m_mainShooterMotorSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim1.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim2.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is
     * updated to match that RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return true.
     */
    @Test
    void testisShooterAtSpeed() {
        m_shooter = new ShooterSubsystem();

        double targetRPM = TEST_RPM;
        m_shooter.setShooterRPM(targetRPM);

        assertTrue(m_shooter.isShooterAtSpeed());
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM but the motors are currently at a different RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return false.
     */
    @Test
    void testFlywheelsNotAtSpeed() {
        m_shooter.setShooterRPM(TEST_RPM);
        SparkFlexSim m_mainShooterMotorSim = new SparkFlexSim(m_shooter.getMainShooterMotor(), null);
        SparkFlexSim m_followerShooterMotorSim1 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(1), null);
        SparkFlexSim m_followerShooterMotorSim2 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(2), null);

        m_mainShooterMotorSim.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_followerShooterMotorSim1.setVelocity(ShooterConstants.kShooterIdleRPM);
        m_followerShooterMotorSim2.setVelocity(ShooterConstants.kShooterIdleRPM);

        assertNotEquals(TEST_RPM, ShooterConstants.kShooterIdleRPM); // sanity check to ensure test values are not equal
        assertFalse(m_shooter.isShooterAtSpeed()); // therefore, this test should assert false
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the {@code stowHood()} method is called and the hood is simulated to be at the stowed position.
     * THEN the {@code isHoodStowed()} method should return true. (Assumption: This tests the sensor logic more than the movement?)
     */
    @Test
    void testHoodStow() {
        m_shooter = new ShooterSubsystem();
        SparkFlex m_hoodMotor = m_shooter.getHoodMotor(); // construct a real motor object to simulate encoder
        SparkAbsoluteEncoderSim m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_hoodMotor); // simulate encoder from real motor
        m_hoodEncoderSim.setPosition(HOOD_START_POSITION);

        m_shooter.stowHood();

        // FIXME 20260224.1233 bbontrager, this test should be isolated
        // by creating a new test specifically for setHoodPosition.
        assertTrue(m_shooter.isHoodStowed()); // the function should return true if...
        assertEquals(m_shooter.getHoodPosition(), ShooterConstants.kHoodStowedPosition); // ... this also asserts true
        // if one assert fails, this whole test should fail.
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position and the simulation is updated to match that position.
     * THEN the {@code isHoodAtPosition()} method should return true for that position.
     */
    @Test
    void testisHoodAtPosition() {
        m_shooter = new ShooterSubsystem();

        // again, simulate the hood motor encoder from the real hood motor controller
        SparkAbsoluteEncoderSim  m_hoodEncoderSim = new SparkAbsoluteEncoderSim(m_shooter.getHoodMotor());

        // use setHoodPosition() to set the position of the hood motor
        m_shooter.setHoodPosition(TARGET_HOOD_POSITION);

        // both assertions should pass to logically complete this test
        // FIXME 20260224.1233 bbontrager, this test should be isolated
        // by creating a new test specifically for setHoodPosition.
        assertTrue(m_shooter.isHoodAtPosition(TARGET_HOOD_POSITION));
        assertEquals(m_shooter.getHoodPosition(), TARGET_HOOD_POSITION);
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

        m_shooter.setHoodPosition(testfail);

        // Check not only isHoodAtPosition returns false,
        // but also hood encoder was set properly using setHoodPosition
        // FIXME 20260224.1233 bbontrager, this test should be isolated
        // by creating a new test specifically for setHoodPosition.
        assertFalse(m_shooter.isHoodAtPosition(TARGET_HOOD_POSITION));
        assertEquals(m_hoodEncoderSim.getPosition(), testfail);
    }

    /**
     * GIVEN a ShooterSubsystem with the hood simulated to be at a known position.
     * WHEN {@code getHoodPosition()} is called.
     * THEN the method should return the known position of the hood.
     */
    @Test
    void testGetHoodPosition() {
        m_shooter = new ShooterSubsystem();

        m_shooter.setHoodPosition(TEST_HOOD_POSITION);

        assertEquals(TEST_HOOD_POSITION, m_shooter.getHoodPosition(), 0.01);
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

        m_shooter.stopHood();

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
        SparkFlexSim m_followerShooterMotorSim1 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(1), null);
        SparkFlexSim m_followerShooterMotorSim2 = new SparkFlexSim(m_shooter.getFollowerShooterMotor(2), null);
        SparkFlexSim m_feederSim = new SparkFlexSim(m_shooter.getFeedMotor(), null);
        SparkFlexSim m_hoodSim = new SparkFlexSim(m_shooter.getHoodMotor(), null);

        m_shooter.setShooterRPM(TEST_RPM);
        m_shooter.manualHood(MANUAL_HOOD_CONTROL_SPEED);
        m_shooter.setFeederRPM(TEST_RPM / 4);

        m_shooter.stopAll();

        assertEquals(0, m_mainShooterMotorSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim1.getAppliedOutput(), TOLERANCE);
        assertEquals(0, m_followerShooterMotorSim2.getAppliedOutput(), TOLERANCE);
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
        SparkClosedLoopController m_feedController = m_shooter.getShooterPID("feed");
        m_shooter.setFeederRPM(TEST_RPM);
        step();

        assertEquals(m_feedController.getControlType(), ControlType.kVelocity);
        assertEquals(m_feedController.getSetpoint(), TEST_RPM, TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the setHoodPosition method is called with a position.
     * THEN the hood's closed-loop controller setpoint should be updated to that position.
     */
    @Test
    void testsetHoodPositionUpdatesSetpoint() {
        double targetPosition = 0.75;
        SparkClosedLoopController hoodController = m_shooter.getShooterPID("hood");

        m_shooter.setHoodPosition(targetPosition);
        step(); // Advance simulation to process the command

        assertEquals(targetPosition, hoodController.getSetpoint(), TOLERANCE);
        assertEquals(ControlType.kMAXMotionPositionControl, hoodController.getControlType());
    }
}
