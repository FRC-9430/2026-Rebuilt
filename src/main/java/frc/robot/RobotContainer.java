// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.util.StatusLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.TunerConstants;
// import frc.robot.Constants.ClimberArmConstants;
// import frc.robot.subsystems.ClimbingArmSubsystem;
import frc.robot.autos.AimAndShootCommand;
import frc.robot.commands.EjectHopperCommand;
import frc.robot.commands.RetractHopperCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootTouchingHubCommand;
import frc.robot.commands.VolleyShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ElasticDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import static frc.robot.Constants.DriveConstants.*;

import java.util.HashMap;

/**
 * Central robot container that creates subsystems, binds controls to commands,
 * and exposes autonomous routines.
 */
public class RobotContainer {

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(kControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    // public final ClimbingArmSubsystem climber = new ClimbingArmSubsystem();

    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public final PolarSubsystem polar = new PolarSubsystem(drivetrain);

    public ElasticDashboard dash = new ElasticDashboard();

    public DriveMode driveMode = DriveMode.CARTESIAN;

    public AimAndShootCommand aimAndShootCommand = new AimAndShootCommand(drivetrain, shooter, intake, polar);

    public ShootCommand shootCommand = new ShootCommand(shooter, polar, intake);
    public VolleyShootCommand volleyCommand = new VolleyShootCommand(shooter, polar, intake);
    public ShootTouchingHubCommand shootTouchingHubCommand = new ShootTouchingHubCommand(shooter, intake);

    /**
     * Construct and configure the robot: set up the drivetrain, dashboard,
     * named commands, and button bindings.
     */
    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        configureNamedCommands();
        dash.initAutoChooser();
        configureBindings();
        StatusLogger.disableAutoLogging();
    }

    /**
     * Configure controller bindings and the drivetrain default command.
     * This sets up button handlers for intake, shooting, mode toggles, and
     * other operator controls.
     */
    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequestWithCondition(
                        () -> (controller.getHID().getRightBumperButton()
                                ? slow
                                        .withVelocityX(-controller.getLeftY() * MaxSpeed / 2.0)
                                        .withVelocityY(-controller.getLeftX() * MaxSpeed / 2.0)
                                        .withRotationalRate(-controller.getRightX() * MaxAngularRate / 1.5)
                                : drive
                                        .withVelocityX(-controller.getLeftY() * MaxSpeed)
                                        .withVelocityY(-controller.getLeftX() * MaxSpeed)
                                        .withRotationalRate(-controller.getRightX() * MaxAngularRate)),
                        () -> aim.withSpeeds(polar.getPolarDriveSpeeds(drivetrain.getState().Pose,
                                (Math.abs(controller.getLeftY()) > 0.06 ? controller.getLeftY() : 0.0) // Clamp Input
                                        / (shootCommand.isScheduled() && polar.targetIsHub() ? 5.0 : 1.0), // Slow When
                                                                                                           // Shooting
                                (Math.abs(controller.getLeftX()) > 0.06 ? controller.getLeftX() : 0.0)
                                        / (shootCommand.isScheduled() && polar.targetIsHub() ? 10.0 : 1.0),
                                MaxSpeed, MaxAngularRate,
                                (false))), // Lead only when shooting
                        () -> isCartesian()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading
        controller.povDown().onTrue(new InstantCommand(() -> {
            var seenPose = vision.getPoseEstimateMT1().pose;
            if (!seenPose.equals(new Pose2d())) {
                drivetrain.resetPose(vision.getPoseEstimateMT1().pose);
            } else {
                drivetrain.seedFieldCentric();
            }

        }));

        // Toggle Polar mode
        controller.leftBumper().onTrue(new InstantCommand(() -> {
            if (isCartesian()) {
                driveMode = DriveMode.POLAR;
            } else {
                driveMode = DriveMode.CARTESIAN;
            }
        }));

        // Shoot
        
        // controller.leftTrigger(0.05).onTrue((polar.targetIsHub()? shootCommand : volleyCommand))
        //         .onFalse(new InstantCommand(() -> shootCommand.cancel()));

        controller.leftTrigger(0.05).onTrue(new InstantCommand(()->{
            if (polar.targetIsHub()) {
                CommandScheduler.getInstance().schedule(shootCommand);
            } else {
                CommandScheduler.getInstance().schedule(shootCommand);
            }
        })).onFalse(new InstantCommand(()->{
            CommandScheduler.getInstance().cancel(shootCommand, volleyCommand);
        }));

        // Intake
        controller.rightTrigger(0.05).whileTrue(new RepeatCommand(new InstantCommand(() -> {
            intake.startIntake();
        }))).onFalse(new InstantCommand(() -> {
            intake.stopAll();
        }));

        // Climber
        // controller.x().onTrue(new InstantCommand(() -> {
        //     CommandScheduler.getInstance().schedule(shootTouchingHubCommand);
        // })).onFalse(new InstantCommand(() -> {
        //     CommandScheduler.getInstance().cancel(shootTouchingHubCommand);
        // }));

        // controller.y().whileTrue(new RepeatCommand(new InstantCommand(() -> {
        //     CommandScheduler.getInstance().cancelAll();
        // }))).onFalse(new InstantCommand(() -> {
        // }));

        // controller.a().onTrue(new InstantCommand(() -> {
        //     SignalLogger.start();
        // })).onFalse(new InstantCommand(() -> {
            
        // }));
        // controller.b().onTrue(new InstantCommand(() -> {
        //     SignalLogger.stop();
        // })).onFalse(new InstantCommand(() -> {
            
        // }));

        // controller.x()
        //         .onTrue(new ShootTouchingHubCommand(shooter, intake))
        //         .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // controller.y().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // Eject Hopper
        controller.start().onTrue(new EjectHopperCommand(intake));

        // // Retract Hopper
        controller.back().onTrue(new RetractHopperCommand(intake));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    /** Drive mode for the operator controls: Cartesian or Polar. */
    public enum DriveMode {
        CARTESIAN,
        POLAR
    }

    /**
     * Returns true when the current drive mode is Cartesian.
     * 
     * @return true if Cartesian drive mode is active
     */
    public boolean isCartesian() {
        return driveMode == DriveMode.CARTESIAN;
    }

    /**
     * Returns true when the current drive mode is Polar.
     * 
     * @return true if Polar drive mode is active
     */
    public boolean isPolar() {
        return driveMode == DriveMode.POLAR;
    }

    /** Set the drive mode to Cartesian. */
    public void setCartesian() {
        driveMode = DriveMode.CARTESIAN;
        SmartDashboard.putString("DriveMode", "CARTESIAN");
    }

    /** Set the drive mode to Polar. */
    public void setPolar() {
        driveMode = DriveMode.POLAR;
        SmartDashboard.putString("DriveMode", "POLAR");
    }

    /**
     * Register Named Commands for PathPlanner and dashboard
     */
    public void configureNamedCommands() {

        HashMap<String, Command> namedCommands = new HashMap<>();

        namedCommands.put("Eject Hopper", new EjectHopperCommand(intake));
        namedCommands.put("Retract Hopper", new RetractHopperCommand(intake));
        namedCommands.put("Start Intake", new InstantCommand(() -> intake.startIntake()));
        namedCommands.put("Stop Intake", new InstantCommand(() -> intake.stopIntake()));
        namedCommands.put("Engage Polar", new InstantCommand(() -> setPolar()));
        namedCommands.put("Engage Cartesian", new InstantCommand(() -> setCartesian()));
        namedCommands.put("Start Aim and Shoot", new InstantCommand(() -> startAimAndShootAutonCommand()));
        namedCommands.put("Aim & Shoot + Timeout", new AimAndShootCommand(drivetrain, shooter, intake, polar));
        namedCommands.put("Stop Aim and Shoot", new InstantCommand(() -> aimAndShootCommand.cancel()));
        namedCommands.put("Stow Hood", new InstantCommand(() -> shooter.stowHood()));
        namedCommands.put("Shoot While Touching Hub", new ShootTouchingHubCommand(shooter, intake));

        namedCommands.put("Stop All Motors & Commands", new InstantCommand(() -> {
            CommandScheduler.getInstance().cancelAll();
            shooter.stopAll();
            intake.stopAll();
        }));

        NamedCommands.registerCommands(namedCommands);
        dash.initCommandChooser(namedCommands);

    }

    /**
     * Returns the currently selected autonomous command from the dashboard
     * chooser.
     * 
     * @return the selected autonomous Command
     */
    public Command getAutonomousCommand() {
        return dash.getAutoChooser();
    }

    /**
     * Schedule the aim-and-shoot autonomous command when in Polar mode
     * during the autonomous period.
     */
    public void startAimAndShootAutonCommand() {

        CommandScheduler.getInstance().schedule(aimAndShootCommand);

    }

}
