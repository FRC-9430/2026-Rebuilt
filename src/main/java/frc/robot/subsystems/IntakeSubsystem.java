// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  final SparkFlex intakeMotor;
  final SparkFlex basketMotor;

  final SparkClosedLoopController intakeController;
  final RelativeEncoder intakeEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor = new SparkFlex(CANConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    basketMotor = new SparkFlex(CANConstants.BASKET_MOTOR_CAN_ID, MotorType.kBrushless);

    intakeMotor.configure(kIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    basketMotor.configure(kBasketMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeController = intakeMotor.getClosedLoopController();
    intakeEncoder = intakeMotor.getEncoder();
  }

  /**
   * Runs the intake at the default speed
   */
  public void setIntake() {
    intakeController.setSetpoint(kDefaultIntakeSpeed, ControlType.kVelocity);
  }

  /**
   * Runs the intake at a specified speed
   * 
   * @param speed The speed to run the intake at
   */
  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Target the intake at a specified RPM
   * 
   * @param RPM The RPM to run the intake at
   */
  public void setIntakeRPM(double RPM) {
    intakeController.setSetpoint(RPM, ControlType.kVelocity);
  }

  /**
   * Stops the intake
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
  
  /**
   * Gets the intake RPM
   */
  public double getIntakeV() {
    return intakeEncoder.getVelocity();
  }

  /**
   * Runs the basket at a specified speed
   * @param speed The speed to run the basket at
   */
  public void setBasket(double speed) {
    basketMotor.set(speed);
  }

  /**
   * Stops the basket
   */
  public void stopBasket() {
    basketMotor.stopMotor();
  }

  /**
   * Stops both the intake and basket motors
   */
  public void stopAll() {
    intakeMotor.stopMotor();
    basketMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake V", intakeEncoder.getVelocity());
  }

  @Override
  public void close() {
    intakeMotor.close();
    basketMotor.close();
  }

}
