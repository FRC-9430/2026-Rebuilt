// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;

public class RobotContainer {

  private CommandXboxController driverController = new CommandXboxController(
      ControllerConstants.DRIVER_CONTROLLER_PORT);

  private Climber robotClimber = new Climber();

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Configure button bindings.
   *
   * <p>
   * Use the command-based framework for button bindings. Example usages:
   *
   * <pre>{@code
   * // Single-shot
   * driverController.a().onTrue(
   *     new InstantCommand(() -> { ... }));
   *
   * // Repeating while held
   * driverController.b().whileTrue(
   *     new RepeatCommand(
   *         new InstantCommand(() -> { ... })));
   *
   * // Axis threshold
   * driverController.rightTrigger(threshold).whileTrue(...);
   * }</pre>
   */
  private void configureBindings() {

    driverController.a().onTrue(new InstantCommand(() -> {
      // Code To Run
    }));

    driverController.b().whileTrue(new RepeatCommand(new InstantCommand(() -> {
      // Code To Run
    })));

    driverController.x()
        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
          robotClimber.setClimberMotor1(1);
        }))).onFalse(new InstantCommand(() -> { robotClimber.stopClimber1(0); }));

    driverController.rightTrigger(/* Threshold */).whileTrue(new RepeatCommand(new InstantCommand(() -> {
      // drivercontroller.getRightTriggerAxis();
    }))).onFalse(new InstantCommand(() -> {
      // Code To Run
    }));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
