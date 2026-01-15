package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_topFlywheelMotor;
  private final SparkFlex m_bottomFlywheelMotor;
  private final SparkFlex m_hoodMotor;

  public ShooterSubsystem() {
    m_topFlywheelMotor = new SparkFlex(ShooterConstants.kShooterLeftMotorId, MotorType.kBrushless);
    m_bottomFlywheelMotor = new SparkFlex(ShooterConstants.kShooterRightMotorId, MotorType.kBrushless);
    m_hoodMotor = new SparkFlex(ShooterConstants.kShooterHoodMotorId, MotorType.kBrushless);

    // TODO: Configure motors

    // TODO: Get encoders

    // TODO: Get PID controllers
  }

  public void setFlywheelSpeeds(double topRPM, double bottomRPM) {
    // TODO: Set flywheel velocity for both motors
  }

  public void idleFlywheels() {
    // TODO: Run at idle RPM
  }

  public void stopFlywheels() {
    // TODO: Stop all flywheel motors
  }

  public boolean flywheelsAtSpeed() {
    // TODO: Check if all flywheels are within tolerance
    return false;
  }

  public double getFlywheelRPM() {
    // TODO: Return average flywheel RPM
    return 0.0;
  }

  public void setShootingAngle(double position) {
    // TODO: Set hood position (clamped to limits)
  }

  public void stowHood() {
    // TODO: Move hood to stowed position
  }

  public void manualHoodControl(double position) {
    // TODO: Manual hood control
  }

  public void stopHood() {
    // TODO: Stop hood motor
  }

  public boolean hoodAtPosition(double position) {
    // TODO: Check if hood is at target position
    return false;
  }

  public boolean isHoodStowed() {
    // TODO: Check if hood is stowed
    return false;
  }

  public double getHoodPosition() {
    // TODO: Return hood position
    return 0.0;
  }

  /**
   * Checks if the flywheels are at their target RPM and the hood is at its target
   * position.
   *
   * @return true if the shooter is ready to fire, false otherwise.
   */
  public boolean isReadyToShoot() {
    return false; // placeholder
  }

  @Override
  public void periodic() {
  }

  public void stopAll() {
    stopFlywheels();
    stopHood();
  }

  public SparkFlex getTopFlywheelMotor() {
    return m_topFlywheelMotor;
  }

  public SparkFlex getBottomFlywheelMotor() {
    return m_bottomFlywheelMotor;
  }

  public SparkFlex getHoodMotor() {
    return m_hoodMotor;
  }
}
