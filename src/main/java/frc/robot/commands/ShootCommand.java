// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.cert.PolicyNode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  final ShooterSubsystem shoot;
  final PolarSubsystem polar;
  final IntakeSubsystem intake;
  double bumpTimer = 0.0;
  double uptime = 0.0;

  /**
   * Creates a new ShootCommand.
   * Sets the shooter speeds and hood angle based on the polar subsystem's
   * calculations. If the shooter is up to speed and the hood is in position, runs
   * the feeder and conveyor to shoot.
   */
  public ShootCommand(ShooterSubsystem shoot, PolarSubsystem polar, IntakeSubsystem intake) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.polar = polar;
    this.intake = intake;
    // SmartDashboard.putNumber("Set Shoot V", 3800);
    // SmartDashboard.putNumber("Set Hood Pos", 0.1);
    // SmartDashboard.putNumber("Set Feed V", 83);
    // SmartDashboard.putNumber("Set Convey V", 1000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot Command Init");
    intake.setHopper(0.08);
    bumpTimer = Timer.getFPGATimestamp();
    uptime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shoot.setShooterRPM(SmartDashboard.getNumber("Set Shoot V", 4000));
    // shoot.setHoodPosition(SmartDashboard.getNumber("Set Hood Pos", 0.1));
    shoot.setShooterRPM(polar.targetIsHub() ? polar.getShootVelocity() : 260 * polar.getRadius() + 2420);
    shoot.setHoodPosition(polar.targetIsHub() ? polar.getHoodPosition() : ShooterConstants.kHoodVollyPosition);
    if (Timer.getFPGATimestamp() > uptime + 0.4) {
      // shoot.setFeederRPS(SmartDashboard.getNumber("Set Feed V", 83));
      shoot.startFeeder();
    }
    if (Timer.getFPGATimestamp() > uptime + 0.6) {
      // shoot.setConveyorRPM(SmartDashboard.getNumber("Set Convey V", 1000));
      shoot.startConveyor();
    }

    if (polar.targetIsHub()) {

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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopHood();
    shoot.stopConveyor();
    shoot.stopFeeder();
    shoot.stopShooter();
    
    if (polar.targetIsHub()) {
      intake.stopHopper();
      intake.stopIntake();
    }

    if (DriverStation.isTeleop()) {
      CommandScheduler.getInstance().schedule(new EjectHopperCommand(intake));

    }

    System.out.println("End Shoot Command: " + interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (DriverStation.isAutonomous() && Timer.getFPGATimestamp() > uptime + 3.3);
  }
}
