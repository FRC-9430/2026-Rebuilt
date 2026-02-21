// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootCommand extends Command {

  final SwerveRequest.ApplyRobotSpeeds aim;
  final CommandSwerveDrivetrain drive;
  final PolarSubsystem polar;
  final double MaxSpeed;
  final double MaxAngularRate;
  final ShooterSubsystem shoot;
  final IntakeSubsystem intake;
  double bumpTimer;

  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand(CommandSwerveDrivetrain drive, ShooterSubsystem shoot, IntakeSubsystem intake,
      PolarSubsystem polar,
      SwerveRequest.ApplyRobotSpeeds aim,
      double MaxSpeed, double MaxAngularRate) {
    addRequirements(drive, shoot, polar);
    this.drive = drive;
    this.shoot = shoot;
    this.polar = polar;
    this.aim = aim;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;
    this.intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoot.setShooterSpeedsRPM(polar.getShootVelocity());
    shoot.setShootingAngle(polar.getHoodPosition());
    if (shoot.isReadyToShoot()) {
      shoot.setFeeder();
      shoot.setConveyor();
    }
    bumpTimer = Timer.getFPGATimestamp();
    System.out.println("AimShoot Command Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.applyRequest(() -> aim.withSpeeds(polar.getPolarDriveSpeeds(drive.getState().Pose,
        0, 0,
        MaxSpeed, MaxAngularRate)));

    shoot.setShooterSpeedsRPM(polar.getShootVelocity());
    shoot.setShootingAngle(polar.getHoodPosition());
    if (shoot.isReadyToShoot()) {
      shoot.setFeeder();
      shoot.setConveyor();
    }
    if (Timer.getFPGATimestamp() - bumpTimer > 1.0) {
      // BumpBasket
      intake.setIntakeRPM(1000);

      if (Timer.getFPGATimestamp() - bumpTimer < 1.2) {
        intake.setBasket(0.15);
      } else {
        intake.setBasket(-0.08);
      }

      if (Timer.getFPGATimestamp() - bumpTimer > 1.4) {
        intake.stopBasket();
        intake.stopIntake();
        bumpTimer = Timer.getFPGATimestamp();
        System.out.println("AimAndShoot - Bumped");
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AimShoot Command End: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
