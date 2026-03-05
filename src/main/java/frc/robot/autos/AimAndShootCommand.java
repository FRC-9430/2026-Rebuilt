// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShootCommand extends SequentialCommandGroup {

  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand(CommandSwerveDrivetrain drive, ShooterSubsystem shoot,
      IntakeSubsystem intake, PolarSubsystem polar) {

    if (DriverStation.isAutonomous()) {
      addCommands( // Auton - Run for 5 seconds
          new ParallelCommandGroup(
              new AimCommand(drive, polar).withTimeout(5.0),
              new ShootCommand(shoot, polar, intake).withTimeout(5.0)));
    } else {
      addCommands( // Teleop - Go Until Cancelled
          new ParallelCommandGroup(
              new AimCommand(drive, polar),
              new ShootCommand(shoot, polar, intake)));
    }
  }

  /** Creates a new AimAndShootCommand with timeout. */
  public AimAndShootCommand(CommandSwerveDrivetrain drive, ShooterSubsystem shoot,
      IntakeSubsystem intake, PolarSubsystem polar, Double timeoutSeconds) {
    System.out.println("New Aim & Shoot Command: " + timeoutSeconds);
    if (timeoutSeconds != null) {
      System.out.println("Timing out");
      addCommands(
          new ParallelCommandGroup(
              new AimCommand(drive, polar).withTimeout(timeoutSeconds),
              new ShootCommand(shoot, polar, intake).withTimeout(timeoutSeconds))
              .withTimeout(timeoutSeconds));
    } else if (DriverStation.isAutonomous()) {
      System.out.println("Auton");
      addCommands( // Auton - Run for 5 seconds
          new ParallelCommandGroup(
              new AimCommand(drive, polar).withTimeout(5.0),
              new ShootCommand(shoot, polar, intake).withTimeout(5.0)));
    } else {
      System.out.println("Teleop");
      addCommands( // Teleop - Go Until Cancelled
          new ParallelCommandGroup(
              new AimCommand(drive, polar),
              new ShootCommand(shoot, polar, intake)));
    }

  }

}
