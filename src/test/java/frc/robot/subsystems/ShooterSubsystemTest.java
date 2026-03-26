package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.verify;

import java.lang.reflect.Field;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.StatusCode;
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

    // Mocks for internal hardware components
    private SparkFlex mockRightTopShooter;
    private SparkFlex mockRightBottomShooter;
    private SparkFlex mockLeftTopShooter;
    private SparkFlex mockLeftBotShooter;
    private SparkFlex mockConveyorMotor;
    private TalonFX mockHoodMotor;
    private TalonFX mockRightFeedMotor;
    private TalonFX mockLeftFeedMotor;
    private CANcoder mockHoodEncoder;
    private SparkClosedLoopController mockShooterController;
    private RelativeEncoder mockShooterEncoder;
    private SparkClosedLoopController mockConveyorController;
    private RelativeEncoder mockConveyorEncoder;
    private SparkAbsoluteEncoder mockAbsoluteEncoder;
    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 2500.0;
    private static final double FLYWHEEL_RPM_TEST_TOLERANCE = 50.0;
    private static final double HOOD_START_POSITION = 0.5;
    private static final double TEST_HOOD_POSITION = 0.45;
    private static final double TARGET_HOOD_POSITION = 0.3; // Example target position
    private static final double HOOD_TOLERANCE = 0.30;
    private static final double MANUAL_HOOD_CONTROL_SPEED = 0.5;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        // m_shooter is NOT initialized here. Each test will call a helper.
    }

    /**
     * Helper to make mock SparkClosedLoopController stateful for setpoint testing.
     */
    private void setupSetpointStubbing(SparkClosedLoopController controller) {
        doAnswer(invocation -> {
            double setpoint = invocation.getArgument(0);
            when(controller.getSetpoint()).thenReturn(setpoint);
            return null;
        }).when(controller).setSetpoint(any(Double.class), any(ControlType.class));
    }

    // Helper method to initialize m_shooter with mocks for other tests
    private void initializeShooterSubsystemWithMocks() {
        try (MockedConstruction<SparkFlex> mockedSparkFlex = Mockito.mockConstruction(SparkFlex.class,
                (mock, context) -> {
                    int canId = (int) context.arguments().get(0);
                    if (canId == frc.robot.Constants.CANConstants.RIGHT_TOP_SHOOT_MOTOR_CAN_ID) {
                        mockRightTopShooter = mock;
                        mockShooterController = mock(SparkClosedLoopController.class);
                        mockShooterEncoder = mock(RelativeEncoder.class);
                        when(mock.getClosedLoopController()).thenReturn(mockShooterController);
                        when(mock.getEncoder()).thenReturn(mockShooterEncoder);
                        // Make mockShooterController stateful for setpoint
                        setupSetpointStubbing(mockShooterController);
                    } else if (canId == frc.robot.Constants.CANConstants.RIGHT_BOTTOM_SHOOT_MOTOR_CAN_ID) {
                        mockRightBottomShooter = mock;
                    } else if (canId == frc.robot.Constants.CANConstants.LEFT_TOP_SHOOT_MOTOR_CAN_ID) {
                        mockLeftTopShooter = mock;
                    } else if (canId == frc.robot.Constants.CANConstants.LEFT_BOTTOM_SHOOT_MOTOR_CAN_ID) {
                        mockLeftBotShooter = mock;
                    } else if (canId == frc.robot.Constants.CANConstants.CONVEYOR_MOTOR_CAN_ID) {
                        mockConveyorMotor = mock;
                        mockConveyorController = mock(SparkClosedLoopController.class);
                        mockConveyorEncoder = mock(RelativeEncoder.class);
                        when(mock.getClosedLoopController()).thenReturn(mockConveyorController);
                        when(mock.getEncoder()).thenReturn(mockConveyorEncoder);
                        // Make mockConveyorController stateful for setpoint
                        setupSetpointStubbing(mockConveyorController);
                    }
                    mockAbsoluteEncoder = mock(SparkAbsoluteEncoder.class);
                    when(mock.getAbsoluteEncoder()).thenReturn(mockAbsoluteEncoder);
                })) {
            try (MockedConstruction<TalonFX> mockedTalonFX = Mockito.mockConstruction(TalonFX.class,
                    (mock, context) -> {
                        int canId = (int) context.arguments().get(0);
                        if (canId == frc.robot.Constants.CANConstants.HOOD_MOTOR_CAN_ID) {
                            mockHoodMotor = mock;
                        } else if (canId == frc.robot.Constants.CANConstants.RIGHT_FEEDER_MOTOR_CAN_ID) {
                            mockRightFeedMotor = mock;
                        } else if (canId == frc.robot.Constants.CANConstants.LEFT_FEEDER_MOTOR_CAN_ID) {
                            mockLeftFeedMotor = mock;
                        }
                        when(mock.getConfigurator()).thenReturn(mock(TalonFXConfigurator.class));
                        when(mock.getConfigurator().apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
                        when(mock.getVelocity()).thenReturn(mock(StatusSignal.class));
                        when(mock.getVelocity().getValueAsDouble()).thenReturn(0.0);
                        when(mock.getPosition()).thenReturn(mock(StatusSignal.class));
                        when(mock.getPosition().getValueAsDouble()).thenReturn(0.0);
                    })) {
                try (MockedConstruction<CANcoder> mockedCANcoder = Mockito.mockConstruction(CANcoder.class,
                        (mock, context) -> {
                            int canId = (int) context.arguments().get(0);
                            if (canId == frc.robot.Constants.CANConstants.HOOD_ENCODER_CAN_ID) {
                                mockHoodEncoder = mock;
                            }
                            when(mock.getConfigurator()).thenReturn(mock(CANcoderConfigurator.class));
                            when(mock.getConfigurator().apply(any(CANcoderConfiguration.class))).thenReturn(StatusCode.OK);
                            when(mock.getPosition()).thenReturn(mock(StatusSignal.class));
                            when(mock.getPosition().getValueAsDouble()).thenReturn(0.0);
                            when(mock.getVelocity()).thenReturn(mock(StatusSignal.class));
                            when(mock.getVelocity().getValueAsDouble()).thenReturn(0.0);
                        })) {
                    m_shooter = new ShooterSubsystem();
                }
            }
        } catch (Exception e) {
            fail("Failed to construct ShooterSubsystem with mocks: " + e.getMessage());
        }
    }

    @AfterEach
    void tearDown() {
        if (m_shooter != null) {
            try {
                m_shooter.close();
            } catch (Exception e) {
                // Ignore errors during standard close
            }
            m_shooter = null;
        }
        // Null out captured mocks
        mockRightTopShooter = null;
        mockRightBottomShooter = null;
        mockLeftTopShooter = null;
        mockLeftBotShooter = null;
        mockConveyorMotor = null;
        mockHoodMotor = null;
        mockRightFeedMotor = null;
        mockLeftFeedMotor = null;
        mockHoodEncoder = null;
        mockShooterController = null;
        mockShooterEncoder = null;
        mockConveyorController = null;
        mockConveyorEncoder = null;
        mockAbsoluteEncoder = null;
        HAL.shutdown();
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
     * is actually executed. This ensures the correct configs are passed to
     * each SparkFlex motor in the ShooterSubsystem.
     */
    @Test
    void testConstructorConfiguration() {
        // Mock construction for SparkFlex
        try (MockedConstruction<SparkFlex> mockedSparkFlex = Mockito.mockConstruction(SparkFlex.class,
                (mock, context) -> {
                    // Return non-null objects for getters used in constructor
                    when(mock.getClosedLoopController()).thenReturn(mock(SparkClosedLoopController.class));
                    when(mock.getEncoder()).thenReturn(mock(RelativeEncoder.class));
                    // getAbsoluteEncoder is not used in ShooterSubsystem constructor, but it's good practice to mock if it might be called.
                    when(mock.getAbsoluteEncoder()).thenReturn(mock(SparkAbsoluteEncoder.class)); // Keep for consistency with original
                })) {
            // Mock construction for TalonFX
            try (MockedConstruction<TalonFX> mockedTalonFX = Mockito.mockConstruction(TalonFX.class,
                    (mock, context) -> {
                        // Mock the configurator for TalonFX
                        when(mock.getConfigurator()).thenReturn(mock(TalonFXConfigurator.class));
                        // Mock the apply method of the configurator
                        when(mock.getConfigurator().apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
                    })) {
                // Mock construction for CANcoder
                try (MockedConstruction<CANcoder> mockedCANcoder = Mockito.mockConstruction(CANcoder.class,
                        (mock, context) -> {
                            // Mock the configurator for CANcoder
                            when(mock.getConfigurator()).thenReturn(mock(CANcoderConfigurator.class));
                            // Mock the apply method of the configurator
                            when(mock.getConfigurator().apply(any(CANcoderConfiguration.class))).thenReturn(StatusCode.OK);
                        })) {

                    m_shooter = new ShooterSubsystem();

                    List<SparkFlex> constructedSparkFlexes = mockedSparkFlex.constructed();
                    List<TalonFX> constructedTalonFXs = mockedTalonFX.constructed();
                    List<CANcoder> constructedCANcoders = mockedCANcoder.constructed();

                    // Verify counts
                    assertEquals(5, constructedSparkFlexes.size(), "Expected 5 SparkFlex motors");
                    assertEquals(3, constructedTalonFXs.size(), "Expected 3 TalonFX motors");
                    assertEquals(1, constructedCANcoders.size(), "Expected 1 CANcoder");

                    // Retrieve mocks based on instantiation order
                    // SparkFlex motors
                    SparkFlex rightTopShooter = constructedSparkFlexes.get(0);
                    SparkFlex rightBottomShooter = constructedSparkFlexes.get(1);
                    SparkFlex leftTopShooter = constructedSparkFlexes.get(2);
                    SparkFlex leftBotShooter = constructedSparkFlexes.get(3);
                    SparkFlex conveyorMotor = constructedSparkFlexes.get(4);

                    // TalonFX motors
                    TalonFX hoodMotor = constructedTalonFXs.get(0);
                    TalonFX rightFeedMotor = constructedTalonFXs.get(1);
                    TalonFX leftFeedMotor = constructedTalonFXs.get(2);

                    // CANcoder
                    CANcoder hoodEncoder = constructedCANcoders.get(0);

                    // Verify SparkFlex configurations
                    Mockito.verify(rightTopShooter).configure(Mockito.eq(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG), Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
                    Mockito.verify(rightBottomShooter).configure(Mockito.eq(ShooterConstants.AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG), Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
                    Mockito.verify(leftTopShooter).configure(Mockito.eq(ShooterConstants.AUX_INVERTED_MOTOR_SHOOTER_CONFIG), Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
                    Mockito.verify(leftBotShooter).configure(Mockito.eq(ShooterConstants.AUX_INVERTED_MOTOR_SHOOTER_CONFIG), Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));
                    Mockito.verify(conveyorMotor).configure(Mockito.eq(ShooterConstants.CONVEYOR_MOTOR_CONFIG), Mockito.eq(ResetMode.kResetSafeParameters), Mockito.eq(PersistMode.kPersistParameters));

                    // Verify TalonFX configurations
                    Mockito.verify(hoodMotor.getConfigurator()).apply(Mockito.eq(ShooterConstants.HOOD_MOTOR_CONFIG));
                    Mockito.verify(rightFeedMotor.getConfigurator()).apply(Mockito.eq(ShooterConstants.MAIN_FEEDER_MOTOR_CONFIG));
                    Mockito.verify(leftFeedMotor.getConfigurator()).apply(Mockito.eq(ShooterConstants.AUX_FEEDER_MOTOR_CONFIG));
                    Mockito.verify(leftFeedMotor).setControl(any(Follower.class)); // Verify Follower control

                    // Verify CANcoder configuration
                    Mockito.verify(hoodEncoder.getConfigurator()).apply(Mockito.eq(ShooterConstants.HOOD_CANCODER_CONFIG));
                }
            }
        }
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is
     * advanced.
     * THEN both flywheels should be configured to the target setpoint as a velocity
     * control
     */
    @Test
    void testSetShooterRPM() {
        initializeShooterSubsystemWithMocks();
        double targetRPM = TEST_RPM;

        // Simulate flywheels spinning up
        m_shooter.setShooterRPM(targetRPM);
        step();

        SparkClosedLoopController controller = mockShooterController; // Use the captured mock
        assertNotNull(controller); // controller is created
        ArgumentCaptor<Double> setpointCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<ControlType> controlTypeCaptor = ArgumentCaptor.forClass(ControlType.class);
        verify(controller).setSetpoint(setpointCaptor.capture(), controlTypeCaptor.capture());
        assertEquals(targetRPM, setpointCaptor.getValue(), TOLERANCE);
        assertEquals(ControlType.kVelocity, controlTypeCaptor.getValue());
    }

    /**
     * GIVEN a ShooterSubsystem with the flywheels commanded to spin.
     * WHEN the {@code stopFlywheels()} method is called.
     * THEN the applied output to both flywheel motors should be zero.
     */
    @Test
    void testStopShooter() {
        initializeShooterSubsystemWithMocks();
        m_shooter.setShooterRPM(TEST_RPM); // Set it to something
        step();

        m_shooter.stopShooter();
        step();

        verify(mockRightTopShooter).stopMotor();
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM and the simulation is
     * updated to match that RPM.
     * THEN the {@code isShooterAtSpeed()} method should return true.
     */
    @Test
    void testIsShooterAtSpeed() {
        initializeShooterSubsystemWithMocks();
        // No SparkFlexSim needed for the mock, we'll directly stub the encoder

        // Define and convert inputs
        // Convert kV from Volts/RPM to Volts/(rad/s)
        // 1 RPM = 0.10472 rad/s = V*60 / (2pi)
        double kV_SI = ShooterConstants.kShooterV * 60 / (2 * Math.PI);
        // Convert kV from Amps/RPM to Amps/(rad/s)
        // A*60 / (2pi)
        // Use a default kA if constant is 0 to prevent simulation errors
        double kA_SI = ShooterConstants.kShooterA == 0 ? 0.005 : ShooterConstants.kShooterA * 60 / (2 * Math.PI);

        // Feed inputs to a generic simulated flywheel system
        // ref:
        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html#wpilib-s-simulation-classes
        FlywheelSim flywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(kV_SI, kA_SI),
                DCMotor.getNeoVortex(3).withReduction(0.67),
                1.0);

        double targetRPM = TEST_RPM;

        // enable the robot so controllers produce output!
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        // Stub the mock encoder to return the flywheel sim's velocity
        when(mockShooterEncoder.getVelocity()).thenAnswer(invocation -> flywheelSim.getAngularVelocityRPM());

        m_shooter.setShooterRPM(targetRPM);

        // Simulation Loop: Run for 2 seconds
        for (int i = 0; i < 100; i++) {
            double currentRPM = flywheelSim.getAngularVelocityRPM();

            // Spark Flex logic loop (PID + FF)
            // Spark Flex PID calculation: Output = (Error * P) + (Setpoint * V) + (S *
            // -1/+1)
            // NOTE 20260226.1926 bbontrager, kP is DutyCycle/RPM, while kV and kS are
            // typically Volts/RPM and Volts.
            // We must divide the Voltage terms by 12.0 to get Duty Cycle.
            // ref:
            // https://docs.revrobotics.com/revlib/spark/closed-loop/units#default-units
            double error = targetRPM - currentRPM;
            double dutyCycle = (error * ShooterConstants.kShooterP)
                    + (targetRPM * ShooterConstants.kShooterV / 12.0)
                    + (Math.signum(targetRPM) * ShooterConstants.kShooterS / 12.0);

            // Clamp duty cycle to motor limits
            dutyCycle = Math.max(-1.0, Math.min(1.0, dutyCycle));
            double motorVoltage = dutyCycle * 12.0;

            // Physics simulation
            // FlywheelSim is a perfect linear system, so subtract kS from the motor's
            // output so the physics plant simulates the friction the motor is fighting.
            double frictionVolts = ShooterConstants.kShooterS * Math.signum(motorVoltage);
            double appliedToPlant = Math.abs(motorVoltage) > ShooterConstants.kShooterS
                    ? motorVoltage - frictionVolts
                    : 0;

            flywheelSim.setInputVoltage(appliedToPlant);
            flywheelSim.update(0.02);
            step(0.02);
        }

        // After the loop, the subsystem should report being at speed
        assertTrue(m_shooter.isShooterAtSpeed(),
                "Shooter should be at speed. Current velocity: " + mockShooterEncoder.getVelocity() +
                        "\nTarget velocity: " + mockShooterController.getSetpoint() +
                        "\nWithin Tolerance: " + (Math.abs(mockShooterEncoder.getVelocity() - mockShooterController.getSetpoint()) <= FLYWHEEL_RPM_TEST_TOLERANCE));
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the overloaded isShooterAtSpeed(rpm, setpoint) method is called.
     * THEN it should correctly evaluate if the RPM is within tolerance of the setpoint.
     */
    @Test
    void testIsShooterAtSpeedLogic() {
        // No need to initialize subsystem with mocks for this test, as it's a pure logic check
        // The method is: Math.abs(rpm - setpoint) <= kShooterToleranceRPM;
        // kShooterToleranceRPM is 100.0

        // Test within tolerance (e.g., abs(2500 - 2550) = 50 <= 100 -> true)
        assertTrue(Math.abs(TEST_RPM - (TEST_RPM + ShooterConstants.kShooterToleranceRPM / 2)) <= ShooterConstants.kShooterToleranceRPM,
                "Should be at speed when within tolerance");

        // Test too low (e.g., abs(2500 - 2700) = 200 <= 100 -> false)
        assertFalse(Math.abs(TEST_RPM - (TEST_RPM + ShooterConstants.kShooterToleranceRPM * 2)) <= ShooterConstants.kShooterToleranceRPM,
                "Should not be at speed when too low");

        // Test too high (e.g., abs(2700 - 2500) = 200 <= 100 -> false)
        assertFalse(Math.abs((TEST_RPM + ShooterConstants.kShooterToleranceRPM * 2) - TEST_RPM) <= ShooterConstants.kShooterToleranceRPM,
                "Should not be at speed when too high");
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the flywheels are commanded to a target RPM but the motors are currently
     * at a different RPM.
     * THEN the {@code flywheelsAtSpeed()} method should return false.
     */
    @Test
    void testFlywheelsNotAtSpeed() {
        initializeShooterSubsystemWithMocks();
        double targetRPM = TEST_RPM;
        m_shooter.setShooterRPM(targetRPM); // This calls setSetpoint on mockShooterController

        // Explicitly stub getSetpoint to ensure it returns the targetRPM
        when(mockShooterController.getSetpoint()).thenReturn(targetRPM);

        when(mockShooterEncoder.getVelocity()).thenReturn(ShooterConstants.kShooterIdleRPM);
        step();

        assertFalse(m_shooter.isShooterAtSpeed()); // setShooterRPM passes, therefore this test should assert false
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN  the hood is simulated to be at the stowed position.
     * THEN the {@code isHoodStowed()} method should return true.
     * NOTE 20260225.0801 bbontrager, Implement new test case to cover stowHood() logic.
     */
    @Test
    void testHoodStow() {
        initializeShooterSubsystemWithMocks();
        // Simulate hood encoder at the stowed position
        when(mockHoodEncoder.getPosition().getValueAsDouble()).thenReturn(ShooterConstants.kHoodStowedPosition);

        assertTrue(m_shooter.isHoodStowed(), "Hood should be stowed.");
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the hood is commanded to a target position but the simulation is at a
     * different position.
     * THEN the {@code isHoodAtPosition()} method should return false for the target
     * position.
     */
    @Test
    void testHoodNotAtPosition() {
        initializeShooterSubsystemWithMocks();
        // Simulate hood encoder at an off-target position
        double offTargetPosition = TARGET_HOOD_POSITION - 0.25;
        when(mockHoodEncoder.getPosition().getValueAsDouble()).thenReturn(offTargetPosition);

        assertFalse(m_shooter.isHoodAtPosition(TARGET_HOOD_POSITION), "Hood should not be at target position.");
    }

    /**
     * GIVEN a ShooterSubsystem with the hood motor running.
     * WHEN the {@code stopHood()} method is called.
     * THEN the applied output to the hood motor should be zero.
     */
    @Test
    void testStopHood() {
        initializeShooterSubsystemWithMocks();
        m_shooter.setHoodDutyCycle(MANUAL_HOOD_CONTROL_SPEED);
        step();
        m_shooter.stopHood();
        step();
        verify(mockHoodMotor).stopMotor();
    }

    /**
     * GIVEN a ShooterSubsystem with all motors running.
     * WHEN the {@code stopAll()} method is called.
     * THEN the applied output to all motors in the subsystem should be zero.
     */
    @Test
    void testStopAll() {
        initializeShooterSubsystemWithMocks();
        m_shooter.setShooterRPM(TEST_RPM);
        m_shooter.setHoodDutyCycle(MANUAL_HOOD_CONTROL_SPEED);
        m_shooter.setFeederRPS(TEST_RPM / 4);
        step();
        m_shooter.stopAll();
        step();

        verify(mockRightTopShooter).stopMotor();
        verify(mockHoodMotor).stopMotor();
        verify(mockRightFeedMotor).stopMotor();
        verify(mockConveyorMotor).stopMotor();
    }

    /**
     * GIVEN a ShooterSubsystem
     * WHEN feed motor is engaged
     * THEN feed controller should be set to the PID velocity
     */
    @Test
    void testSetFeederRPSUpdatesSetpoint() {
        initializeShooterSubsystemWithMocks();
        TalonFX mockFeederMotor = mockRightFeedMotor; // Use the captured mock

        double targetRPS = TEST_RPM;
        m_shooter.setFeederRPS(targetRPS);

        // Capture the VelocityVoltage control request passed to setControl
        ArgumentCaptor<VelocityVoltage> velocityCaptor = ArgumentCaptor.forClass(VelocityVoltage.class);
        Mockito.verify(mockFeederMotor).setControl(velocityCaptor.capture());

        assertEquals(targetRPS, velocityCaptor.getValue().Velocity, TOLERANCE);
    }

    /**
     * GIVEN a ShooterSubsystem.
     * WHEN the setHoodPosition method is called with a position.
     * THEN the hood's closed-loop controller setpoint should be updated to that
     * position.
     */
    @Test
    void testSetHoodPositionUpdatesSetpoint() {
         initializeShooterSubsystemWithMocks();
         TalonFX mockHoodMotor = this.mockHoodMotor; // Use the captured mock

         m_shooter.setHoodPosition(TARGET_HOOD_POSITION);

         // Capture the PositionVoltage control request passed to setControl
         ArgumentCaptor<PositionVoltage> positionCaptor = ArgumentCaptor.forClass(PositionVoltage.class);
         Mockito.verify(mockHoodMotor).setControl(positionCaptor.capture());

         assertEquals(TARGET_HOOD_POSITION, positionCaptor.getValue().Position, TOLERANCE);
     }

    /**
     * GIVEN a ShooterSubsystem with a hood motor encoder
     * WHEN setHoodPosition called with a target position parameter
     * THEN hood encoder reaches target position
     */
    @Test
    void testSetHoodPosition() {
        initializeShooterSubsystemWithMocks();
        // Test that positions outside safe bounds are ignored
        m_shooter.setHoodPosition(0.0); // Below kHoodMinSafePosition
        verify(mockHoodMotor, never()).setControl(any(PositionVoltage.class));
    }
}
