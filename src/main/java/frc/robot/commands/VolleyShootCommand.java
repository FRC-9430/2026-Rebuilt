// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VolleyShootCommand extends Command {

  final ShooterSubsystem shoot;
  final IntakeSubsystem intake;
  final PolarSubsystem polar;
  double uptime = 0.0;

  /** Creates a new VolleyShootCommand. */
  public VolleyShootCommand(ShooterSubsystem shoot, PolarSubsystem polar, IntakeSubsystem intake) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.polar = polar;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    uptime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shoot.setShooterRPM(260 * polar.getRadius() + 2420);
    shoot.setHoodPosition(ShooterConstants.kHoodVollyPosition);

    if (Timer.getFPGATimestamp() > uptime + 0.2) {
      shoot.startFeeder();
    }

    if (Timer.getFPGATimestamp() > uptime + 0.3) {
      shoot.startConveyor();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopHood();
    shoot.stopConveyor();
    shoot.stopFeeder();
    shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
