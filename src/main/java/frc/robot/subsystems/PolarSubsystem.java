// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.FieldConstants.*;

public class PolarSubsystem extends SubsystemBase {

  public final CommandSwerveDrivetrain driveTrain;

  public Translation2d target;
  public double radiusToTarget = 0.0;

  /** Creates a new PolarSubsystem. */
  public PolarSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.driveTrain = drivetrain;

    // Determine target point depending on alliance
    switch (DriverStation.getAlliance().get()) {
      case Red:
        // Red alliance Hub
        target = RED_HUB_LOC;
        break;
      case Blue:
        // Blue alliance Hub
        target = BLUE_HUB_LOC;
        break;
      default:
        // Fallback to a reasonable default on unknown alliance
        target = BLUE_HUB_LOC;
        break;
    }

  }

  public void setTarget(Translation2d newTarget) {
    target = newTarget;
  }

  public double getShootVelocity() {
    return Math.floor(PolarUtils.getEstShootVelFrmR(radiusToTarget));
  }

  public double getHoodPosition() {
    return Math.floor(1000.0*PolarUtils.getEstShootVelFrmR(radiusToTarget))/1000.0;
  }

  public void calculateRadius() {
    radiusToTarget = driveTrain.getPose().getTranslation().getDistance(target);
  }

  public ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, double radialIn, double orbitalIn, double MaxSpeed,
      double MaxAngularRate) {
    return PolarUtils.getPolarDriveSpeeds(estPose, target, radialIn, orbitalIn, MaxSpeed, MaxAngularRate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateRadius();

  }

  public class PolarUtils {

    public static ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, Translation2d target, double radialIn,
        double orbitalIn, double MaxSpeed, double MaxAngularRate) {

      Translation2d pose = estPose.getTranslation();

      double radialInput = -radialIn;
      double orbitInput = -orbitalIn;

      // Clamp Input
      radialInput = (Math.abs(radialInput) > 0.06 ? radialInput : 0.0);
      orbitInput = (Math.abs(orbitInput) > 0.06 ? orbitInput : 0.0);

      // Compute vector from robot to the target (field frame)
      double dx = target.getX() - pose.getX();
      double dy = target.getY() - pose.getY();
      double r = Math.hypot(dx, dy);

      r = getRadiusFrom(pose, target);

      SmartDashboard.putNumber("Dist From Hub", r);

      // If very close to the point, avoid divide-by-zero and simply rotate in place
      // to face the point
      // Not likely to come up in practice, just a fail safe
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
      double currentAngle = estPose.getRotation().getRadians();
      // Normalize angle error to [-pi, pi]
      double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));

      // P = 20
      double omega = 15 * angleError;

      // Clamp angular rate to configured maximum
      omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));

      // Convert field-relative velocities to robot-relative chassis speeds
      return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, -omega, estPose.getRotation());
    }

    public static double getRadiusFrom(Translation2d pose, Translation2d target) {
      return pose.getDistance(target);
    }

    public static double getEstHoodFrmR(double r) {
      double pos = -0.0200985 * Math.pow(r, 2)
          + 0.168875 * r
          + (-0.099575 + Constants.ShooterConstants.kHoodMinPosition);
      SmartDashboard.putNumber("Calc Hood Pos", pos);
      return pos;
    }

    public static double getEstShootVelFrmR(double r) {
      double rpm = -2.10158 * Math.pow(r, 4)
          + 24.78781 * Math.pow(r, 3)
          + -79.29767 * Math.pow(r, 2)
          + 213.13977 * r
          + 2440.02268;
      SmartDashboard.putNumber("Calc Shoot V", rpm);
      return rpm;
    }
  }
}
