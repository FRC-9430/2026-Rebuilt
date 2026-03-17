// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BumpHopperCommand extends Command {

  final IntakeSubsystem intake;
  double startTime;
  double totalTimes;
  double bumps;

  /**
   * Creates a new BumpHoppperCommand.
   * Bumps the hoppper back and forth push balls into the shooter.
   * 
   * @param times The number of times to bump the hoppper.
   */
  public BumpHopperCommand(IntakeSubsystem intake, int times) {
    addRequirements(intake);
    this.intake = intake;
    this.totalTimes = times;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setHopper(0.08);
    intake.setIntakeRPM(1000);
    startTime = Timer.getFPGATimestamp();
    bumps = 0;
    System.out.println("Bump Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > startTime + 0.5) {

      System.out.println("Bump: " + bumps);
      startTime = Timer.getFPGATimestamp();
      intake.setHopper(0.15);
        bumps++;
    } else {
      if (Timer.getFPGATimestamp() > startTime + 0.25) {
        intake.setHopper(-0.15);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopHopper();
    intake.stopIntake();
    System.out.println("Bump Hopper Ended: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return totalTimes <= bumps;
  }
}
