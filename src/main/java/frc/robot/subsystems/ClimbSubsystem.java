// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  private AbsoluteEncoder ClimberEncoder;   
  private PIDController climberController = new PIDController (PIDConstants.kClimberkp, PIDConstants.kClimberki, PIDConstants.kClimberkd);


















  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     climberController.calculate(ClimberEncoder.getPosition(), 0.5);
  }
}
