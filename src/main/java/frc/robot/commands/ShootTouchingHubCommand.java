// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootTouchingHubCommand extends Command {

  final ShooterSubsystem shoot;
  final IntakeSubsystem intake;
  double bumpTimer = 0.0;
  double uptime = 0.0;

  /**
   * Shoots at the hub only when touching hub, times out after 5 seconds
   */
  public ShootTouchingHubCommand(ShooterSubsystem shoot, IntakeSubsystem intake) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot&Touch Command Init");
    intake.setBasket(0.08);
    bumpTimer = Timer.getFPGATimestamp();
    uptime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setShooterRPM(2750);
    shoot.setHoodPosition(0.415);
    if (shoot.isShooterReady() && Timer.getFPGATimestamp() < uptime + 0.6) {
      shoot.startFeeder();
      shoot.startConveyorDefault();
    }

    if (intake.getIntakeV() < 1000)
      intake.setIntakeRPM(1000);

    double cur = Timer.getFPGATimestamp();
    if (cur - bumpTimer < 0.25) {
      intake.setBasket(0.12);
    } else if (cur - bumpTimer < 0.5) {
      intake.setBasket(-0.12);
    } else {
      bumpTimer = Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stowHood();
    shoot.stopConveyor();
    shoot.stopFeeder();
    shoot.stopShooter();
    intake.stopBasket();
    intake.stopIntake();
    System.out.println("End Shoot&Touch Command: " + interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > uptime + 5);
  }
}
