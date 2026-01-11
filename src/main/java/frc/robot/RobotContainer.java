// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ProtoShooter;

public class RobotContainer {

  CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  ProtoShooter protoShooter = new ProtoShooter();

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {

    driverController.povUp().onTrue(new InstantCommand(() -> {
      protoShooter.setMainSpeed(protoShooter.mainSpeed + 0.05);
    }));

    driverController.povDown().onTrue(new InstantCommand(() -> {
      protoShooter.setMainSpeed(protoShooter.mainSpeed - 0.05);
    }));

    driverController.povRight().onTrue(new InstantCommand(() -> {
      protoShooter.setSubSpeed(protoShooter.subSpeed + 0.05);
    }));

    driverController.povLeft().onTrue(new InstantCommand(() -> {
      protoShooter.setSubSpeed(protoShooter.subSpeed - 0.05);
    }));

    driverController.leftTrigger(0.5).whileTrue(new InstantCommand(() -> {
      protoShooter.runBackward();
    })).onFalse(new InstantCommand(() -> {
      protoShooter.stopMotors();
    }));

    driverController.rightTrigger(0.5).whileTrue(new InstantCommand(() -> {
      protoShooter.runForward();
    })).onFalse(new InstantCommand(() -> {
      protoShooter.stopMotors();
    }));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
