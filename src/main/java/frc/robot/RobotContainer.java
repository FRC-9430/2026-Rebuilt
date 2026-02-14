// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.util.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ElasticDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    // Proportional gain to rotate the robot to face the target while in POLAR mode.
    // Tweak this value to change how aggressively the robot turns to face the point.
    private double kOrientationP = 4.0;

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

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();    public final VisionSubsystem vision = new VisionSubsystem();

    public ElasticDashboard dash = new ElasticDashboard();

    public DriveMode driveMode = DriveMode.CARTESIAN;

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
                () -> aim.withSpeeds(getPolarDriveSpeeds()), 
                ()->isCartesian()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(()->idle).ignoringDisable(true));

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
            SmartDashboard.putString("DriveMode", "POLAR");
        })).onFalse(new InstantCommand(() -> {
            driveMode = DriveMode.CARTESIAN;
            SmartDashboard.putString("DriveMode", "CARTESIAN");
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

    public boolean isCartesian() {
        return driveMode == DriveMode.CARTESIAN;
    }

    public void setInitialPose() {
        drivetrain.resetPose(dash.getInitialPose());
    }

    public ChassisSpeeds getPolarDriveSpeeds() {

        // Determine target point depending on alliance
        double targetPointX;
        double targetPointY;
        switch (DriverStation.getAlliance().get()) {
            case Red:
                // Red alliance Hub
                targetPointX = 11.9;
                targetPointY = 4.0;
                break;
            case Blue:
                // Blue alliance Hub
                targetPointX = 4.625;
                targetPointY = 4.0;
                break;
            default:
                // Fallback to a reasonable default on unknown alliance
                targetPointX = 4.625;
                targetPointY = 4.0;
                break;
        }

        Pose2d pose = drivetrain.getState().Pose;

        // Controller mapping: forward/back controls radial motion toward/away from the
        // point.
        // Left stick: negative Y is forward on this controller mapping in this project,
        // so we negate
        double radialInput = -controller.getLeftY(); // +1 => forward (towards point)
        // Left stick left/right will orbit around the point
        double orbitInput = -controller.getLeftX();

        radialInput = (Math.abs(radialInput) > 0.05 ? radialInput : 0);
        orbitInput = (Math.abs(orbitInput) > 0.05 ? orbitInput : 0);

        // Compute vector from robot to the target (field frame)
        double dx = targetPointX - pose.getX();
        double dy = targetPointY - pose.getY();
        double r = Math.hypot(dx, dy);

        // If very close to the point, avoid divide-by-zero and simply rotate in place
        // to face the point
        final double kEpsilon = 1e-3;
        if (r < kEpsilon) {
            // If we're essentially on the point, don't translate; just rotate to face the
            // (same) point (zero)
            return new ChassisSpeeds();
        }

        // Unit radial vector (points from robot toward the target)
        double ux = dx / r;
        double uy = dy / r;
        // Unit tangential vector (90 deg CCW from radial) to produce orbiting motion
        double tx = -uy; // -dy/r
        double ty = ux; // dx/r

        // Map inputs to speeds. Use MaxSpeed for both radial and tangential components.
        double vRadial = radialInput * MaxSpeed; // toward/away
        double vTangential = orbitInput * MaxSpeed; // orbit speed

        // Compose field-relative linear velocity: radial component + tangential
        // component
        double vxField = vRadial * ux + vTangential * tx;
        double vyField = vRadial * uy + vTangential * ty;

        // To keep the robot pointed at the target, compute heading error and use a
        // P-controller to rotate the robot to face the target. This makes the
        // robot's orientation converge to the angle toward the target regardless
        // of tangential motion.
        double desiredAngle = Math.atan2(dy, dx);
        double currentAngle = pose.getRotation().getRadians();
        // Normalize angle error to [-pi, pi]
        double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));

        double omega = kOrientationP * angleError;
        // Clamp angular rate to configured maximum
        omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));

        // Convert field-relative velocities to robot-relative chassis speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omega, pose.getRotation());
    }

    public void addVisionMeasurements() {
        var poseEstimate = vision.getPoseEstimate();
        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimate.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimate.pose.getY());

        boolean doRejectUpdate = false;
        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
            if (poseEstimate.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (poseEstimate.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (poseEstimate.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds);
        }

    }

}
