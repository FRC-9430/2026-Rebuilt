// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PolarSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {

  final SwerveRequest.ApplyRobotSpeeds aim;
  final CommandSwerveDrivetrain drive;
  final PolarSubsystem polar;
  final double MaxSpeed;
  final double MaxAngularRate;
  double uptime;

  /** Creates a new AimCommand. 
   * Runs until interrupted. 
   * Aims robot at polar target */
  public AimCommand(CommandSwerveDrivetrain drive, PolarSubsystem polar) {
    //addRequirements(drive);
    this.drive = drive;
    this.polar = polar;
    this.aim = DriveConstants.aim;
    this.MaxSpeed = DriveConstants.MaxSpeed;
    this.MaxAngularRate = DriveConstants.MaxAngularRate;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Aim Command Init");
    uptime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveRobotRelative(polar.getPolarDriveSpeeds(drive.getState().Pose,
        0, 0,
        MaxSpeed, MaxAngularRate, false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Aim Command End: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (DriverStation.isAutonomous() && Timer.getFPGATimestamp() > uptime + 3.3);
  }
}
