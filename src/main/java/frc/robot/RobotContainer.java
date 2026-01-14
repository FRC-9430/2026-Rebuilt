// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.IntakeSubsystem;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  IntakeSubsystem intake = new IntakeSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.rightTrigger(0.08).whileTrue(new RepeatCommand(new InstantCommand(()->{
    intake.setSpeed(controller.getRightTriggerAxis());
   })
        
   ));
   
        

   controller.leftTrigger(0.08).whileTrue(new RepeatCommand(new InstantCommand(()->{
    intake.setSpeed(-controller.getLeftTriggerAxis());
   })
        
   ));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
