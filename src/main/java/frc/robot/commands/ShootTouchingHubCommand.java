// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    intake.setHopper(0.08);
    bumpTimer = Timer.getFPGATimestamp();
    uptime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setShooterRPM(3200);
    shoot.setHoodPosition(0.07);

    if (Timer.getFPGATimestamp() > uptime + 0.2) {
      // shoot.setFeederRPS(SmartDashboard.getNumber("Set Feed V", 83));
      shoot.startFeeder();
    }
    
    if (Timer.getFPGATimestamp() > uptime + 0.3) {
      // shoot.setConveyorRPM(SmartDashboard.getNumber("Set Convey V", 1000));
      shoot.startConveyor();
    }

    if (intake.getIntakeV() < 20)
      intake.setIntakeRPS(20);

    if (Timer.getFPGATimestamp() > bumpTimer + 0.6) {
      intake.setHopper(0.18);
    }

    if (Timer.getFPGATimestamp() > bumpTimer + 2.0) {
      intake.setHopper(-0.18);
      bumpTimer = Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopHood();
    shoot.stopConveyor();
    shoot.stopFeeder();
    shoot.stopShooter();
    intake.stopHopper();
    intake.stopIntake();
    System.out.println("End Shoot&Touch Command: " + interrupted);
    
    CommandScheduler.getInstance().schedule(new EjectHopperCommand(intake));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > uptime + 5);
  }
}
