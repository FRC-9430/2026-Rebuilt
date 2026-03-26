package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

import java.lang.reflect.Field;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;

import static frc.robot.Constants.IntakeConstants.*;

class IntakeSubsystemTest {
    IntakeSubsystem subsystem;

    // Mocks for internal hardware components
    private TalonFX mockIntakeMotor;
    private SparkFlex mockHopperMotor;
    private TalonFXConfigurator mockIntakeConfigurator;
    private StatusSignal<AngularVelocity> mockIntakeVelocitySignal;

    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 1500.0;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        initializeIntakeSubsystemWithMocks();
    }

    // Helper method to initialize subsystem with mocks for other tests
    private void initializeIntakeSubsystemWithMocks() {
        try (MockedConstruction<TalonFX> mockedTalonFX = Mockito.mockConstruction(TalonFX.class,
                (mock, context) -> {
                    int canId = (int) context.arguments().get(0);
                    if (canId == frc.robot.Constants.CANConstants.INTAKE_MOTOR_CAN_ID) {
                        mockIntakeMotor = mock;
                        mockIntakeConfigurator = mock(TalonFXConfigurator.class);
                        mockIntakeVelocitySignal = (StatusSignal<AngularVelocity>) mock(StatusSignal.class);
                        when(mock.getConfigurator()).thenReturn(mockIntakeConfigurator);
                        when(mockIntakeConfigurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
                        when(mock.getVelocity()).thenReturn(mockIntakeVelocitySignal);
                        when(mockIntakeVelocitySignal.getValueAsDouble()).thenReturn(0.0);
                    }
                })) {
            try (MockedConstruction<SparkFlex> mockedSparkFlex = Mockito.mockConstruction(SparkFlex.class,
                    (mock, context) -> {
                        int canId = (int) context.arguments().get(0);
                        if (canId == frc.robot.Constants.CANConstants.HOPPER_MOTOR_CAN_ID) {
                            mockHopperMotor = mock;
                            // Stub configure method
                            when(mock.configure(any(), any(ResetMode.class), any(PersistMode.class))).thenReturn(com.revrobotics.REVLibError.kOk);
                            when(mock.get()).thenReturn(0.0); // Default applied output
                        }
                    })) {
                subsystem = new IntakeSubsystem();
            }
        } catch (Exception e) {
            fail("Failed to construct IntakeSubsystem with mocks: " + e.getMessage());
        }
    }

    @AfterEach
    void tearDown() {
        if (subsystem != null) {
            subsystem.close();
        }
        subsystem = null;
        // Null out captured mocks
        mockIntakeMotor = null;
        mockHopperMotor = null;
        mockIntakeConfigurator = null;
        mockIntakeVelocitySignal = null;
        HAL.shutdown();
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    @Test
    void testMotorsInitialized() {
        assertNotNull(mockIntakeMotor);
        assertNotNull(mockHopperMotor);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntakeRPS is called with a specific speed
     * THEN the intake motor should be commanded to that velocity using VelocityVoltage control
     */
    @Test
    void testSetIntakeRPSUpdatesSetpoint() {
        // subsystem is already initialized with mocks in setUp()

        double targetRPS = TEST_RPM;
        subsystem.setIntakeRPS(targetRPS);

        // Capture the VelocityVoltage control request passed to setControl
        ArgumentCaptor<VelocityVoltage> velocityCaptor = ArgumentCaptor.forClass(VelocityVoltage.class);
        verify(mockIntakeMotor).setControl(velocityCaptor.capture());
        assertEquals(targetRPS, velocityCaptor.getValue().Velocity, TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with the intake running
     * WHEN stopIntake is called
     * THEN the intake motor should be stopped (speed 0)
     */
    @Test
    void testStopIntake() {
        // subsystem is already initialized with mocks in setUp()
        subsystem.setIntakeRPS(0.6);
        step();

        subsystem.stopIntake();
        step();

        verify(mockIntakeMotor).stopMotor();
    }

    /**
     * GIVEN IntakeSubsystem with both motors running
     * WHEN stopAll is called
     * THEN both the intake and basket motors should be stopped
     */
    @Test
    void testStopAll() {
        // subsystem is already initialized with mocks in setUp()
        subsystem.setIntakeRPS(0.6);
        subsystem.setHopper(0.4);
        step();

        subsystem.stopAll();
        step();

        verify(mockIntakeMotor).stopMotor();
        verify(mockHopperMotor).stopMotor();
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setBasket is called with a specific speed
     * THEN the basket motor should be set to that speed
     */
    @Test
    void testSetBasketOpenLoop() {
        // subsystem is already initialized with mocks in setUp()
        double speed = -0.4;
        subsystem.setHopper(speed);
        step();

        verify(mockHopperMotor).set(speed);
    }

    /**
     * GIVEN IntakeSubsystem with the basket running
     * WHEN stopBasket is called
     * THEN the basket motor should be stopped (speed 0)
     */
    @Test
    void testStopBasket() {
        // subsystem is already initialized with mocks in setUp()
        subsystem.setHopper(0.7);
        step();

        subsystem.stopHopper();
        step();

        verify(mockHopperMotor).stopMotor();
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntakeRPS is called with a specific speed
     * THEN the intake motor should be set to that speed
     */
    @Test
    void testStartIntakeUsesDefaultSpeed() {
        // subsystem is already initialized with mocks in setUp()

        // Mock DriverStation to simulate autonomous mode for kDefaultAutoIntakeSpeed
        try (var mockedDS = Mockito.mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::isAutonomous).thenReturn(true); // Simulate autonomous

            subsystem.startIntake();

            // Capture the VelocityVoltage control request passed to setControl
            ArgumentCaptor<VelocityVoltage> velocityCaptor = ArgumentCaptor.forClass(VelocityVoltage.class);
            verify(mockIntakeMotor).setControl(velocityCaptor.capture());

            assertEquals(kDefaultAutoIntakeSpeed, velocityCaptor.getValue().Velocity, TOLERANCE);
        }
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN periodic is called
     * THEN the intake velocity should be published to SmartDashboard
     */
    @Test
    void testPeriodic() throws NoSuchFieldException, IllegalAccessException {
        // subsystem is already initialized with mocks in setUp()
        // Stub the captured mockIntakeVelocitySignal
        when(mockIntakeMotor.getVelocity().getValueAsDouble()).thenReturn(123.45);
        subsystem.periodic();
        assertEquals(123.45, SmartDashboard.getNumber("Intake V", 999), TOLERANCE);
    }
}
