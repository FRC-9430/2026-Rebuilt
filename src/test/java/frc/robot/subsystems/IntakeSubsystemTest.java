package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
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

import static frc.robot.Constants.IntakeConstants.*;

class IntakeSubsystemTest {
    IntakeSubsystem subsystem;

    private SparkFlexSim m_intakeSim;
    private SparkFlexSim m_basketSim;
    private SparkRelativeEncoderSim m_intakeEncoderSim;

    private static final double TOLERANCE = 0.01;
    private static final double TEST_RPM = 1500.0;

    @BeforeEach
    void setUp() {
        // Initialize the HAL to support hardware simulation
        assertTrue(HAL.initialize(500, 0));
        subsystem = new IntakeSubsystem();

        m_intakeSim = new SparkFlexSim(subsystem.m_intakeMotor, null);
        m_basketSim = new SparkFlexSim(subsystem.m_hopperMotor, null);
        m_intakeEncoderSim = new SparkRelativeEncoderSim(subsystem.m_intakeMotor);
    }

    @AfterEach
    void tearDown() {
        // Close the subsystem to release hardware resources (SparkFlex objects)
        if (subsystem != null) {
            subsystem.close();
        }
        subsystem = null;
        m_intakeSim = null;
        m_basketSim = null;
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

    @Test
    void testSetIntakeOpenLoop() {
        double speed = 0.5;
        subsystem.setIntake(speed);
        step();

        assertEquals(speed, subsystem.m_intakeMotor.get(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with the intake running
     * WHEN stopIntake is called
     * THEN the intake motor should be stopped (speed 0)
     */
    @Test
    void testStopIntake() {
        subsystem.setIntake(0.6);
        step();

        subsystem.stopIntake();
        step();

        assertEquals(0.0, m_intakeSim.getAppliedOutput(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem with both motors running
     * WHEN stopAll is called
     * THEN both the intake and basket motors should be stopped
     */
    @Test
    void testStopAll() {
        subsystem.setIntake(0.6);
        subsystem.setHopper(0.4);
        step();

        subsystem.stopAll();
        step();

        assertEquals(0.0, m_intakeSim.getAppliedOutput(), TOLERANCE);
        assertEquals(0.0, m_basketSim.getAppliedOutput(), TOLERANCE);
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
     * WHEN setIntake is called with a specific speed
     * THEN the intake motor should be set to that speed
     */
    @Test
    void testSetIntakeRPMUpdatesController() {
        SparkClosedLoopController intakeController = subsystem.m_intakeController;

        subsystem.setIntakeRPS(TEST_RPM);
        step();

        assertEquals(ControlType.kVelocity, intakeController.getControlType());
        assertEquals(TEST_RPM, intakeController.getSetpoint(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN setIntake is called with no speed
     * THEN the intake motor should be set to default speed
     */
    @Test
    void testSetIntakeDefaultUsesController() {
        SparkClosedLoopController intakeController = subsystem.m_intakeController;

        subsystem.startIntake();
        step();

        assertEquals(ControlType.kVelocity, intakeController.getControlType());
        assertEquals(kDefaultAutoIntakeSpeed, intakeController.getSetpoint(), TOLERANCE);
    }

    /**
     * GIVEN IntakeSubsystem
     * WHEN periodic is called
     * THEN the intake velocity should be published to SmartDashboard
     */
    @Test
    void testPeriodic() {
        subsystem.periodic();
        assertEquals(0.0, SmartDashboard.getNumber("Intake V", 999));
    }
}
