// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.util.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyRobotSpeeds aim = new SwerveRequest.ApplyRobotSpeeds();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public DriveMode driveMode = DriveMode.CARTESIAN;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
                        .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
                ));

        drivetrain.setDefaultCommand(
                new ConditionalCommand(
                        drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y
                                                                                                             // (forward)
                                .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                              // with negative X (left)
                        ),
                        drivetrain.applyRequest(() -> aim.withSpeeds(getPolarDriveSpeeds())),
                        () -> {
                            return driveMode == DriveMode.CARTESIAN;
                        }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        controller.rightBumper().onTrue(new InstantCommand(() -> {
            driveMode = DriveMode.POLAR;
        })).onFalse(new InstantCommand(() -> {
            driveMode = DriveMode.CARTESIAN;
        }));

        drivetrain.registerTelemetry(logger::telemeterize);
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

    public enum DriveMode {
        CARTESIAN,
        POLAR
    }

    public ChassisSpeeds getPolarDriveSpeeds() {

        // Determine target point depending on alliance
        double targetPointX;
        double targetPointY;
        switch (DriverStation.getAlliance().get()) {
            case Red:
                // Red alliance target (near our wall)
                targetPointX = 4.25;
                targetPointY = 4.0;
                break;
            case Blue:
                // Blue alliance target (other side)
                targetPointX = 12.5;
                targetPointY = 4.0;
                break;
            default:
                // Fallback to a reasonable default on unknown alliance
                targetPointX = 4.25;
                targetPointY = 4.0;
                break;
        }

        // Read the robot pose. Use the drivetrain's samplePoseAt with current time
        // If unavailable, stop motion (safe fallback)
        var maybePose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());
        if (maybePose.isEmpty()) {
            return new ChassisSpeeds();
        }

        Pose2d pose = maybePose.get();

        // Controller mapping: forward/back controls radial motion toward/away from the point.
        // Left stick: negative Y is forward on this controller mapping in this project, so we negate
        double radialInput = -controller.getLeftY(); // +1 => forward (towards point)
        // Left stick left/right will orbit around the point
        double orbitInput = -controller.getLeftX();

        // Compute vector from robot to the target (field frame)
        double dx = targetPointX - pose.getX();
        double dy = targetPointY - pose.getY();
        double r = Math.hypot(dx, dy);

        // If very close to the point, avoid divide-by-zero and simply rotate in place to face the point
        final double kEpsilon = 1e-3;
        if (r < kEpsilon) {
            // If we're essentially on the point, don't translate; just rotate to face the (same) point (zero)
            return new ChassisSpeeds();
        }

        // Unit radial vector (points from robot toward the target)
        double ux = dx / r;
        double uy = dy / r;
        // Unit tangential vector (90 deg CCW from radial) to produce orbiting motion
        double tx = -uy; // -dy/r
        double ty = ux;  // dx/r

        // Map inputs to speeds. Use MaxSpeed for both radial and tangential components.
        double vRadial = Math.max(-1.0, Math.min(1.0, radialInput)) * MaxSpeed; // toward/away
        double vTangential = Math.max(-1.0, Math.min(1.0, orbitInput)) * MaxSpeed; // orbit speed

        // Compose field-relative linear velocity: radial component + tangential component
        double vxField = vRadial * ux + vTangential * tx;
        double vyField = vRadial * uy + vTangential * ty;

        // To keep the robot pointed at the target, set angular velocity to the rate of change of the angle
        // to the target: phi_dot = -v_tangential / r (see derivation). Use omega = phi_dot.
        double omega = 0.0;
        if (Math.abs(r) > kEpsilon) {
            omega = -vTangential / r;
        }

        // Convert field-relative velocities to robot-relative chassis speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omega, pose.getRotation());
    }
}
