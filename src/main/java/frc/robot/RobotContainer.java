// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.util.TunerConstants;
import frc.robot.commands.BumpBasketCommand;
import frc.robot.commands.EjectBasketCommand;
import frc.robot.commands.RetractBasketCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ElasticDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.ApplyRobotSpeeds aim = new SwerveRequest.ApplyRobotSpeeds();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public final PolarSubsystem polar = new PolarSubsystem(drivetrain);

    public ElasticDashboard dash = new ElasticDashboard();

    public DriveMode driveMode = DriveMode.CARTESIAN;

    public enum DriveMode {
        CARTESIAN,
        POLAR
    }

    public boolean isCartesian() {
        return driveMode == DriveMode.CARTESIAN;
    }

    public RobotContainer() {
        configureBindings();
        drivetrain.configureAutoBuilder();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequestWithCondition(
                        () -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                                .withRotationalRate(controller.getRightX() * MaxAngularRate),
                        () -> aim.withSpeeds(polar.getPolarDriveSpeeds(drivetrain.getState().Pose,
                                controller.getLeftY(), controller.getLeftX(),
                                MaxSpeed, MaxAngularRate)),
                        () -> isCartesian()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller.povDown().onTrue(new InstantCommand(() -> {
            drivetrain.resetPose(vision.getPoseEstimateMT1().pose);
        }));
        drivetrain.applyRequest(() -> idle).ignoringDisable(true);

        controller.leftBumper().onTrue(new InstantCommand(() -> {
            if (isCartesian()) {
                driveMode = DriveMode.POLAR;
                SmartDashboard.putString("DriveMode", "POLAR");
            } else {
                driveMode = DriveMode.CARTESIAN;
                SmartDashboard.putString("DriveMode", "CARTESIAN");
            }
        }));

        controller.leftTrigger(0.05).whileTrue(new RepeatCommand(new InstantCommand(() -> {

            shooter.setShooterSpeedsRPM(polar.getShootVelocity());
            shooter.setShootingAngle(polar.getHoodPosition());

            if (shooter.isReadyToShoot()) {
                shooter.setFeeder();
                shooter.setConveyor();
            }

        }))).onFalse(new InstantCommand(() -> {
            shooter.stopAll();
        }));

        controller.rightTrigger(0.05).whileTrue(new RepeatCommand(new InstantCommand(() -> {
            intake.setIntake();
        }))).onFalse(new InstantCommand(() -> {
            intake.stopAll();
        }));

        controller.a().onTrue(new InstantCommand(() -> {
            shooter.stowHood();
        })).onFalse(new InstantCommand(() -> {
            shooter.stopHood();
        }));

        controller.y().onTrue(new BumpBasketCommand(intake));

        controller.back().onTrue(new EjectBasketCommand(intake));

        controller.start().onTrue(new RetractBasketCommand(intake));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public void setInitialPose() {
        drivetrain.resetPose(dash.getInitialPose());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

}
