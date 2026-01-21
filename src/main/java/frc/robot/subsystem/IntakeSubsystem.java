// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  SparkFlex intakeMotor = new SparkFlex(33, MotorType.kBrushless);
  SparkFlex conveyorMotor = new SparkFlex(34, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
    conveyorMotor.set(speed);
  }

  /** Closes the motor controllers. Useful for testing cleanup. */
  public void close() {
    intakeMotor.close();
    conveyorMotor.close();
  }
}
