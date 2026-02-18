// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  SparkFlex conveyorMotor = new SparkFlex(CANConstants.CONVEYOR_MOTOR_CAN_ID, MotorType.kBrushless);
  SparkFlex intakeMotor = new SparkFlex(CANConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  SparkFlex basketMotor = new SparkFlex(CANConstants.BASKET_CAN_ID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.configure(kIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    conveyorMotor.configure(kConveyorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    basketMotor.configure(kBasketMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the intake at the default speed
   */
  public void runIntake() {
    intakeMotor.set(kDefaultIntakeSpeed);
  }

  /**
   * Runs the intake at a specified speed
   * 
   * @param speed The speed to run the intake at
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  /**
   * Runs the conveyor at the default speed
   */
  public void runConveyor() {
    conveyorMotor.set(kDefaultConveyorSpeed);
  }

  /**
   * Runs the conveyor at a specified speed
   * 
   * @param speed The speed to run the conveyor at
   */
  public void runConveyor(double speed) {
    conveyorMotor.set(speed);
  }

  /**
   * Stops the conveyor
   */
  public void stopConveyor() {
    conveyorMotor.stopMotor();
  }

  public void setBasket(double speed) {
    basketMotor.set(speed);
  }

  public void stopBasket() {
    basketMotor.stopMotor();
  }

  /**
   * Runs both the intake and conveyor at specified speeds
   * 
   * @param intakeSpeed
   * @param conveyorSpeed
   */
  public void setSpeeds(double intakeSpeed, double conveyorSpeed) {
    intakeMotor.set(intakeSpeed);
    conveyorMotor.set(conveyorSpeed);
  }

  /**
   * Stops both the intake and conveyor
   */
  public void stopAll() {
    intakeMotor.stopMotor();
    conveyorMotor.stopMotor();
    basketMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
