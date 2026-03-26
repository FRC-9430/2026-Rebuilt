package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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

import static frc.robot.Constants.IntakeConstants.*;

class IntakeSubsystemTest {
    IntakeSubsystem subsystem;

    private TalonFXSimState m_intakeSim;
    private TalonFXSimState m_intakeEncoderSim;

    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 1500.0;

    @BeforeEach
    void setUp() {
        // Initialize the HAL to support hardware simulation
        assertTrue(HAL.initialize(500, 0));
        subsystem = new IntakeSubsystem();

        m_intakeSim = new TalonFXSimState(subsystem.m_intakeMotor, null);
        m_intakeEncoderSim = new TalonFXSimState(subsystem.m_intakeMotor);
    }

    @AfterEach
    void tearDown() {
        // Close the subsystem to release hardware resources (SparkFlex objects)
        if (subsystem != null) {
            subsystem.close();
        }
        subsystem = null;
        m_intakeEncoderSim = null;
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    @Test
    void testMotorsInitialized() {
        assertNotNull(subsystem.m_intakeMotor);
        assertNotNull(subsystem.m_hopperMotor);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntakeRPS is called with a specific speed
     * THEN the intake motor should be commanded to that velocity using VelocityVoltage control
     */
    @Test
    void testSetIntakeRPSUpdatesSetpoint() throws NoSuchFieldException, IllegalAccessException {
        // Replace the real TalonFX with a mock for verification
        TalonFX mockIntakeMotor = mock(TalonFX.class);
        Field intakeMotorField = IntakeSubsystem.class.getDeclaredField("m_intakeMotor");
        intakeMotorField.setAccessible(true);
        intakeMotorField.set(subsystem, mockIntakeMotor);

        // Mock the configurator for the mockIntakeMotor
        when(mockIntakeMotor.getConfigurator()).thenReturn(mock(TalonFXConfigurator.class));
        when(mockIntakeMotor.getConfigurator().apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);

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
        subsystem.setIntakeRPS(0.6);
        step();

        subsystem.stopIntake();
        step();

        assertEquals(0.0, m_intakeSim.getTorqueCurrent(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with both motors running
     * WHEN stopAll is called
     * THEN both the intake and basket motors should be stopped
     */
    @Test
    void testStopAll() {
        subsystem.setIntakeRPS(0.6);
        SparkFlexSim m_basketSim = new SparkFlexSim(subsystem.m_hopperMotor, null);
        subsystem.setHopper(0.4);
        step();

        subsystem.stopAll();
        step();

        assertEquals(0.0, m_intakeSim.getSupplyCurrent(), TOLERANCE);
        assertEquals(0.0, m_basketSim.getAppliedOutput(), TOLERANCE); // Now m_basketSim is initialized correctly
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setBasket is called with a specific speed
     * THEN the basket motor should be set to that speed
     */
    @Test
    void testSetBasketOpenLoop() {
        double speed = -0.4;
        subsystem.setHopper(speed);
        step();

        assertEquals(speed, subsystem.m_hopperMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with the basket running
     * WHEN stopBasket is called
     * THEN the basket motor should be stopped (speed 0)
     */
    @Test
    void testStopBasket() {
        subsystem.setHopper(0.7);
        step();

        subsystem.stopHopper();
        step();

        assertEquals(0.0, subsystem.m_hopperMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntakeRPS is called with a specific speed
     * THEN the intake motor should be set to that speed
     */
    @Test
    void testStartIntakeUsesDefaultSpeed() throws NoSuchFieldException, IllegalAccessException {
        // Replace the real TalonFX with a mock for verification
        TalonFX mockIntakeMotor = mock(TalonFX.class);
        Field intakeMotorField = IntakeSubsystem.class.getDeclaredField("m_intakeMotor");
        intakeMotorField.setAccessible(true);
        intakeMotorField.set(subsystem, mockIntakeMotor);

        // Mock the configurator for the mockIntakeMotor
        when(mockIntakeMotor.getConfigurator()).thenReturn(mock(TalonFXConfigurator.class));
        when(mockIntakeMotor.getConfigurator().apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);

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
        // Mock the intake motor to control its velocity for testing SmartDashboard output
        TalonFX mockIntakeMotor = mock(TalonFX.class);
        Field intakeMotorField = IntakeSubsystem.class.getDeclaredField("m_intakeMotor");
        intakeMotorField.setAccessible(true);
        intakeMotorField.set(subsystem, mockIntakeMotor);

        // Mock the getVelocity().getValueAsDouble() call
        when(mockIntakeMotor.getVelocity()).thenReturn(mock(StatusSignal.class));
        when(mockIntakeMotor.getVelocity().getValueAsDouble()).thenReturn(123.45);

        subsystem.periodic();
        assertEquals(123.45, SmartDashboard.getNumber("Intake V", 999), TOLERANCE);
    }
}
