package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.lang.reflect.Field;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

class IntakeSubsystemTest {
    private IntakeSubsystem m_intake;
    private static final double TOLERANCE = 0.01;

    @BeforeEach
    void setUp() {
        // Initialize the HAL to support hardware simulation
        assert HAL.initialize(500, 0);
    }

    @AfterEach
    void tearDown() {
        if (m_intake != null) {
            try {
                m_intake.close();
            } catch (Exception e) {
                // Ignore errors during standard close
            }

            // Use reflection to close all SparkFlex fields to ensure HAL resources are
            // released.
            // This prevents IllegalStateException in subsequent tests if the subsystem's
            // close()
            // method misses any motors.
            try {
                for (Field field : m_intake.getClass().getDeclaredFields()) {
                    if (AutoCloseable.class.isAssignableFrom(field.getType())) {
                        field.setAccessible(true);
                        AutoCloseable ac = (AutoCloseable) field.get(m_intake);
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
            m_intake = null;
        }
    }

    private void step() {
        SimHooks.stepTiming(0.02);
    }

    private void step(double stepSeconds) {
        SimHooks.stepTiming(stepSeconds);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntake is called with a specific speed
     * THEN the intake motor should be set to that speed
     */
    @Test
    void testSetIntake() {
        // Use real objects (Simulated by HAL) to verify motor output
        m_intake = new IntakeSubsystem();
        step();

        m_intake.setIntake(0.5);
        step();

        assertEquals(0.5, m_intake.m_intakeMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with the intake running
     * WHEN stopIntake is called
     * THEN the intake motor should be stopped (speed 0)
     */
    @Test
    void testStopIntake() {
        m_intake = new IntakeSubsystem();

        m_intake.setIntake(0.5);
        step();

        m_intake.stopIntake();
        step();

        assertEquals(0.0, m_intake.m_intakeMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setBasket is called with a specific speed
     * THEN the basket motor should be set to that speed
     */
    @Test
    void testSetBasket() {
        m_intake = new IntakeSubsystem();

        m_intake.setBasket(0.8);
        step();

        assertEquals(0.8, m_intake.m_basketMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with the basket running
     * WHEN stopBasket is called
     * THEN the basket motor should be stopped (speed 0)
     */
    @Test
    void testStopBasket() {
        m_intake = new IntakeSubsystem();

        m_intake.setBasket(0.8);
        step();

        m_intake.stopBasket();
        step();

        assertEquals(0.0, m_intake.m_basketMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with both motors running
     * WHEN stopAll is called
     * THEN both the intake and basket motors should be stopped
     */
    @Test
    void testStopAll() {
        m_intake = new IntakeSubsystem();

        m_intake.setIntake(0.5);
        m_intake.setBasket(0.5);
        step();

        m_intake.stopAll();
        step();

        assertEquals(0.0, m_intake.m_intakeMotor.get(), TOLERANCE);
        assertEquals(0.0, m_intake.m_basketMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN an IntakeSubsystem with mocked hardware
     * WHEN setIntakeRPM is called with a target RPM
     * THEN the closed-loop controller should set the reference to that RPM
     */
    @Test
    void testSetIntakeRPM() {
        // Use Mockito to verify interaction with the closed loop controller
        SparkFlex intakeMock = mock(SparkFlex.class);
        SparkFlex basketMock = mock(SparkFlex.class);
        SparkClosedLoopController controllerMock = mock(SparkClosedLoopController.class);
        RelativeEncoder encoderMock = mock(RelativeEncoder.class);

        when(intakeMock.getClosedLoopController()).thenReturn(controllerMock);
        when(intakeMock.getEncoder()).thenReturn(encoderMock);

        m_intake = new IntakeSubsystem(intakeMock, basketMock); // See? Easier to mock stuff.
        m_intake.setIntakeRPM(2000);

        verify(controllerMock).setSetpoint(2000, ControlType.kVelocity);
    }

    /**
     * GIVEN an IntakeSubsystem with mocked hardware
     * WHEN setIntake is called without arguments
     * THEN the closed-loop controller should set the reference to the default velocity
     */
    @Test
    void testSetIntakeDefault() {
        SparkFlex intakeMock = mock(SparkFlex.class);
        SparkFlex basketMock = mock(SparkFlex.class);
        SparkClosedLoopController controllerMock = mock(SparkClosedLoopController.class);
        RelativeEncoder encoderMock = mock(RelativeEncoder.class);

        when(intakeMock.getClosedLoopController()).thenReturn(controllerMock);
        when(intakeMock.getEncoder()).thenReturn(encoderMock);

        m_intake = new IntakeSubsystem(intakeMock, basketMock);
        m_intake.setIntake();

        verify(controllerMock).setSetpoint(anyDouble(), eq(ControlType.kVelocity));
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN periodic is called
     * THEN the intake velocity should be published to SmartDashboard
     */
    @Test
    void testPeriodic() {
        m_intake = new IntakeSubsystem();
        m_intake.periodic();
        assertEquals(0.0, SmartDashboard.getNumber("Intake V", 999));
    }
}
