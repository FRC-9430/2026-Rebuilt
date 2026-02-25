package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
    private IntakeSubsystem m_intake;

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

    @Test
    @Disabled
    void testIntakeAndConveyorRunTogether() {
        // Engage the intake
        // subsystem.setSpeeds(0.5, 0.5);

        // Verify both motors are set to the same speed
        assertEquals(0.5, m_intake.intakeMotor.get(), 0.001, "Intake motor should be at 0.5 speed");
        // assertEquals(0.5, subsystem.conveyorMotor.get(), 0.001, "Conveyor motor
        // should be at 0.5 speed");
    }

    @Test
    @Disabled
    void testStopDisengagesBoth() {
        m_intake.stopAll();
        assertEquals(0.0, m_intake.intakeMotor.get(), 0.001, "Intake motor should be stopped");
        // assertEquals(0.0, subsystem.conveyorMotor.get(), 0.001, "Conveyor motor
        // should be stopped");
    }
}
