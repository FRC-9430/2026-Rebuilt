// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.TunerConstants;
import frc.robot.autos.AimAndShootCommand;
import frc.robot.commands.BumpBasketCommand;
import frc.robot.commands.EjectBasketCommand;
import frc.robot.commands.RetractBasketCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimbingArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ElasticDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PolarSubsystem;
import static frc.robot.Constants.DriveConstants.*;

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
    public final ClimbingArmSubsystem climber = new ClimbingArmSubsystem();

    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public final PolarSubsystem polar = new PolarSubsystem(drivetrain);

    public ElasticDashboard dash = new ElasticDashboard();

    public DriveMode driveMode = DriveMode.CARTESIAN;

    public AimAndShootCommand aimAndShootCommand = new AimAndShootCommand(drivetrain, shooter, intake, polar);
    public ShootCommand shootCommand = new ShootCommand(shooter, polar);

    /**
     * Construct and configure the robot: set up the drivetrain, dashboard,
     * named commands, and button bindings.
     */
    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        configureNamedCommands();
        dash.initAutoChooser();
        configureBindings();
    }

    /**
     * Configure controller bindings and the drivetrain default command.
     * This sets up button handlers for intake, shooting, mode toggles, and
     * other operator controls.
     */
    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequestWithCondition(
                        () -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate),
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
        // controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
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
                SmartDashboard.putString("DriveMode", "POLAR");
            } else {
                driveMode = DriveMode.CARTESIAN;
                SmartDashboard.putString("DriveMode", "CARTESIAN");
            }
        }));

        // Shoot
        controller.leftTrigger(0.05).onTrue(shootCommand)
                .onFalse(new InstantCommand(() -> shootCommand.cancel()));

        // Intake
        controller.rightTrigger(0.05).whileTrue(new RepeatCommand(new InstantCommand(() -> {
            intake.setIntake();
        }))).onFalse(new InstantCommand(() -> {
            intake.stopAll();
        }));

        // Climber
        controller.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            climber.setClimberMotors(1.0);
        }))).onFalse(new InstantCommand(() -> {
            climber.stopClimbers();
        }));

        controller.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            climber.setClimberMotors(-1.0);
        }))).onFalse(new InstantCommand(() -> {
            climber.stopClimbers();
        }));

        // Force Stow Hood
        controller.a().onTrue(new InstantCommand(() -> {
            shooter.stowHood();
        })).onFalse(new InstantCommand(() -> {
            shooter.stopHood();
        }));

        // Bump the Basket
        controller.y().onTrue(new BumpBasketCommand(intake, 3));

        // Eject Basket
        controller.back().onTrue(new EjectBasketCommand(intake));

        // Retract Basket
        controller.start().onTrue(new RetractBasketCommand(intake));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    /**
     * Reset the drivetrain pose using the initial pose supplied on the dashboard.
     */
    public void setInitialPose() {
        drivetrain.resetPose(dash.getInitialPose());
    }

    /** Drive mode for the operator controls: Cartesian or Polar. */
    public enum DriveMode {
        CARTESIAN,
        POLAR
    }

    /**
     * Returns true when the current drive mode is Cartesian.
     * @return true if Cartesian drive mode is active
     */
    public boolean isCartesian() {
        return driveMode == DriveMode.CARTESIAN;
    }

    /**
     * Returns true when the current drive mode is Polar.
     * @return true if Polar drive mode is active
     */
    public boolean isPolar() {
        return driveMode == DriveMode.POLAR;
    }

    /** Set the drive mode to Cartesian. */
    public void setCartesian() {
        driveMode = DriveMode.CARTESIAN;
    }

    /** Set the drive mode to Polar. */
    public void setPolar() {
        driveMode = DriveMode.POLAR;
    }

    /**
     * Register named commands for PathPlanner
     */
    public void configureNamedCommands() {
        NamedCommands.registerCommand("Eject Basket", new EjectBasketCommand(intake));

        NamedCommands.registerCommand("Retract Basket", new RetractBasketCommand(intake));
        NamedCommands.registerCommand("Bump Basket", new BumpBasketCommand(intake, 1));

        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> intake.setIntake()));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.stopIntake()));

        NamedCommands.registerCommand("Engage Polar", new InstantCommand(() -> setPolar()));
        NamedCommands.registerCommand("Engage Cartesian", new InstantCommand(() -> setCartesian()));

        NamedCommands.registerCommand("Start Aim and Shoot", new InstantCommand(() -> startAimAndShootAutonCommand()));

        NamedCommands.registerCommand("Stop Aim and Shoot", new InstantCommand(() -> aimAndShootCommand.cancel()));

        NamedCommands.registerCommand("Stow Hood", new InstantCommand(() -> shooter.stowHood()));

    }

    /**
     * Returns the currently selected autonomous command from the dashboard
     * chooser.
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
        if (driveMode == DriveMode.POLAR && DriverStation.isAutonomous()) {
            CommandScheduler.getInstance().schedule(aimAndShootCommand);
        }
    }

}
