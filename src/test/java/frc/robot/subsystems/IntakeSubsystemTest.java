package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
  IntakeSubsystem subsystem;

  @BeforeEach
  void setUp() {
    // Initialize the HAL to support hardware simulation
    assert HAL.initialize(500, 0);
    subsystem = new IntakeSubsystem();
  }

  @AfterEach
  void tearDown() {
    // Close the subsystem to release hardware resources (SparkFlex objects)
    subsystem.close();
  }

  @Test
  void testIntakeAndConveyorRunTogether() {
    // Engage the intake
    subsystem.setSpeed(0.5);

    // Verify both motors are set to the same speed
    assertEquals(0.5, subsystem.intakeMotor.get(), 0.001, "Intake motor should be at 0.5 speed");
    assertEquals(0.5, subsystem.conveyorMotor.get(), 0.001, "Conveyor motor should be at 0.5 speed");
  }

  @Test
  void testStopDisengagesBoth() {
    subsystem.setSpeed(0.0);
    assertEquals(0.0, subsystem.intakeMotor.get(), 0.001, "Intake motor should be stopped");
    assertEquals(0.0, subsystem.conveyorMotor.get(), 0.001, "Conveyor motor should be stopped");
  }
}
