// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BumpBasketCommand;
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
      addCommands( // Auton - Run for 4 seconds
          new ParallelCommandGroup(
              new AimCommand(drive, polar).withTimeout(4.0),
              new BumpBasketCommand(intake, 8).withTimeout(4.0),
              new ShootCommand(shoot, polar, intake).withTimeout(4.0)));
    } else {
      addCommands( // Teleop - Go Until Cancelled
          new ParallelCommandGroup(
              new AimCommand(drive, polar),
              new BumpBasketCommand(intake, 8),
              new ShootCommand(shoot, polar, intake)));
    }
  }

}
