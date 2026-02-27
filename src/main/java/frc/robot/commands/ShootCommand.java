// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PolarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  final ShooterSubsystem shoot;
  final PolarSubsystem polar;
  double delay;

  /**
   * Creates a new ShootCommand.
   * Sets the shooter speeds and hood angle based on the polar subsystem's
   * calculations. If the shooter is up to speed and the hood is in position, runs
   * the feeder and conveyor to shoot.
   */
  public ShootCommand(ShooterSubsystem shoot, PolarSubsystem polar) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.polar = polar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot Command Init");
    delay = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setShooterRPM(polar.getShootVelocity());
    shoot.setHoodPosition(polar.getHoodPosition());
    if (shoot.isShooterReady() && polar.Angled() && Timer.getFPGATimestamp() > delay + 0.5) {
      shoot.startFeeder();
      shoot.startConveyorDefault();
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stowHood();
    shoot.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
