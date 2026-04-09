// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectHopperCommand extends Command {

  final IntakeSubsystem intake;
  double startTime;

  /**
   * Creates a new EjectHopperCommand.
   * Ejects the hopper, takes 0.4 seconds to run
   */
  public EjectHopperCommand(IntakeSubsystem intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();

    intake.setHopper(-0.3);

    if (DriverStation.isTeleop()) {
      intake.setIntakeRPS(10);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopHopper();

    if (DriverStation.isTeleop()) {
      intake.stopIntake();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > (startTime + 0.4)); // Run 0.4 second
  }
}
