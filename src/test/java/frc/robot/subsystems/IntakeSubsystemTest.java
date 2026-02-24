package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
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

    m_intakeSim = new SparkFlexSim(subsystem.intakeMotor, null);
    m_basketSim = new SparkFlexSim(subsystem.basketMotor, null);
    m_intakeEncoderSim = new SparkRelativeEncoderSim(subsystem.intakeMotor);
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
    assertNotNull(subsystem.intakeMotor);
    assertNotNull(subsystem.basketMotor);
  }

  @Test
  void testSetIntakeOpenLoop() {
    double speed = 0.5;
    subsystem.setIntake(speed);
    step();

    assertEquals(speed, subsystem.intakeMotor.get(), TOLERANCE);
  }

  @Test
  void testStopIntake() {
    subsystem.setIntake(0.6);
    step();

    subsystem.stopIntake();
    step();

    assertEquals(0.0, m_intakeSim.getAppliedOutput(), TOLERANCE);
  }

  @Test
  void testStopAll() {
    subsystem.setIntake(0.6);
    subsystem.setBasket(0.4);
    step();

    subsystem.stopAll();
    step();

    assertEquals(0.0, m_intakeSim.getAppliedOutput(), TOLERANCE);
    assertEquals(0.0, m_basketSim.getAppliedOutput(), TOLERANCE);
  }

  @Test
  void testSetBasketOpenLoop() {
    double speed = -0.4;
    subsystem.setBasket(speed);
    step();

    assertEquals(speed, subsystem.basketMotor.get(), TOLERANCE);
  }

  @Test
  void testStopBasket() {
    subsystem.setBasket(0.7);
    step();

    subsystem.stopBasket();
    step();

    assertEquals(0.0, subsystem.basketMotor.get(), TOLERANCE);
  }

  @Test
  void testSetIntakeRPMUpdatesController() {
    SparkClosedLoopController intakeController = subsystem.intakeController;

    subsystem.setIntakeRPM(TEST_RPM);
    step();

    assertEquals(ControlType.kVelocity, intakeController.getControlType());
    assertEquals(TEST_RPM, intakeController.getSetpoint(), TOLERANCE);
  }

  @Test
  void testSetIntakeDefaultUsesController() {
    SparkClosedLoopController intakeController = subsystem.intakeController;

    subsystem.setIntake();
    step();

    assertEquals(ControlType.kVelocity, intakeController.getControlType());
    assertEquals(kDefaultIntakeSpeed, intakeController.getSetpoint(), TOLERANCE);
  }
}
