// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.FieldConstants.*;

public class PolarSubsystem extends SubsystemBase {

  public final CommandSwerveDrivetrain driveTrain;

  public Translation2d target;
  public double radiusToTarget = 0.0;

  public enum Mode {
    HUB,
    VOLLEY
  }

  private Mode mode = Mode.HUB;
  /** Default angular tolerance (radians) to consider "facing the target" */
  private double facingToleranceRad = 0.06; // ~3.4 degrees
  /** Estimated conversion from shooter RPM -> projectile speed (m/s). */
  private double rpmToMps = 0.0008;

  /** Creates a new PolarSubsystem. */
  public PolarSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.driveTrain = drivetrain;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        target = BLUE_HUB_LOC;
      } else {
        target = RED_HUB_LOC;
      }
    } else {
      target = BLUE_HUB_LOC;
    }

  }

  /**
   * Sets the robots polar target
   *
   * @param newTarget Translation2d of new target
   */
  public void setTarget(Translation2d newTarget) {
    target = newTarget;
  }

  /**
   * Returns when the robot is moving < 0.1 Rad/s
   *
   * @return slow rotation
   */
  public boolean Angled() {
    return Math.abs(driveTrain.getState().Speeds.omegaRadiansPerSecond) < 0.1;
  }

  /**
   * Calculates and returns the target Shooter Velocity in RPM.
   * When in Volley mode always returns 3000 RPM.
   * Else if the robot is stationary returns the RPM from the derived equation.
   * Else if moving radially to target calculate RPM using an estimated shot lead.
   *
   * @return Target Shooter RPM
   */
  public double getShootVelocity() {
    if (mode == Mode.VOLLEY)
      return 3000.0;

    double vRadial = getRadialV();
    double r = getRadius();

    // Ignore small velocities
    if (Math.abs(vRadial) < 0.05)
      return Math.floor(PolarUtils.getEstShootVelFrmR(r));

    // Compute a dynamic lead time based on estimated projectile speed (from RPM).
    double estRPM = PolarUtils.getEstShootVelFrmR(r);
    double projectileSpeed = Math.max(0.1, estRPM * rpmToMps); // m/s
    double flightTime = r / projectileSpeed;

    double adjustedR = Math.max(0.0, r + vRadial * flightTime);
    return Math.floor(PolarUtils.getEstShootVelFrmR(adjustedR));
  }

  /**
   * Calculates and returns the target Hood position.
   * When in Volley mode always returns 0.8.
   * Else if the robot is stationary returns the position from the derived
   * equation.
   * Else if moving radially to target calculate position using an estimated shot
   * lead.
   *
   * @return Target Hood Position
   */
  public double getHoodPosition() {
    if (mode == Mode.VOLLEY)
      return 0.8;

    double vRadial = getRadialV();
    double r = getRadius();

    // Ignore small velocities
    if (Math.abs(vRadial) < 0.05)
      return Math.floor(1000.0 * PolarUtils.getEstHoodPosFrmR(r)) / 1000.0;

    // Compute same dynamic lead for shoot direction
    double estRPM = PolarUtils.getEstShootVelFrmR(r);
    double projectileSpeed = Math.max(0.1, estRPM * rpmToMps); // m/s
    double flightTime = r / projectileSpeed;
    double adjustedR = Math.max(0.0, r + vRadial * flightTime);

    return Math.floor(1000.0 * PolarUtils.getEstHoodPosFrmR(adjustedR)) / 1000.0;
  }

  /**
   * Returns the radius to target
   *
   * @return radius
   */
  public double getRadius() {
    double r = driveTrain.getPose().getTranslation().getDistance(target);
    SmartDashboard.putNumber("Polar/Distance From Target", r);
    return r;
  }

  /**
   * Calculates and return the radial velocity of the robot around the target hub.
   * Positive when moving away from target.
   *
   * @return Radial Velocity
   */
  public double getRadialV() {
    var pose = driveTrain.getPose();
    var robotTrans = pose.getTranslation();
    double dx = target.getX() - robotTrans.getX();
    double dy = target.getY() - robotTrans.getY();
    double r = Math.hypot(dx, dy);

    var speeds = driveTrain.getState().Speeds;
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;

    double ux = (r > 1e-6) ? dx / r : 0.0;
    double uy = (r > 1e-6) ? dy / r : 0.0;
    double vRadial = vx * ux + vy * uy;

    SmartDashboard.putNumber("Polar/vRadial", vRadial);

    return vRadial;
  }

  /**
   * Calculates and return the orbital velocity of the robot around the target hub.
   *
   * @return Orbital Velocity
   */
  public double getOrbitalV() {
    var pose = driveTrain.getPose();
    var robotTrans = pose.getTranslation();
    double dx = target.getX() - robotTrans.getX();
    double dy = target.getY() - robotTrans.getY();
    double r = Math.hypot(dx, dy);

    var speeds = driveTrain.getState().Speeds;
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;

    double ux = (r > 1e-6) ? dx / r : 0.0;
    double uy = (r > 1e-6) ? dy / r : 0.0;
    double tx = -uy;
    double ty = ux;

    double vOrbital = vx * tx + vy * ty;

    SmartDashboard.putNumber("Polar/vOrbital", vOrbital);

    return vOrbital;
  }

  /**
   * Returns true when the robot is facing the target (within default tolerance).
   */
  public boolean isFacingTarget() {
    return isFacingTarget(facingToleranceRad);
  }

  /**
   * Returns true when the robot is facing the target within the specified
   * tolerance (radians).
   */
  public boolean isFacingTarget(double toleranceRad) {
    var pose = driveTrain.getPose();
    var robotTrans = pose.getTranslation();
    double dx = target.getX() - robotTrans.getX();
    double dy = target.getY() - robotTrans.getY();

    double desiredAngle = Math.atan2(dy, dx);
    double currentAngle = pose.getRotation().getRadians();
    double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));

    return Math.abs(angleError) <= toleranceRad;
  }

  public ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, double radialIn, double orbitalIn, double MaxSpeed,
      double MaxAngularRate, boolean doLeadShot) {
    double orbital = orbitalIn;
    double radial = radialIn;
    if (mode == Mode.VOLLEY) {
      orbital = -orbitalIn;
      radial = -radialIn;
    }

    double estRPM = PolarUtils.getEstShootVelFrmR(getRadius());
    double projectileSpeed = Math.max(0.1, estRPM * rpmToMps); // m/s
    double flightTime = getRadius() / projectileSpeed;

    return PolarUtils.getPolarDriveSpeeds(estPose, target, radial, orbital, MaxSpeed, MaxAngularRate,
        doLeadShot ? getOrbitalV() * flightTime * (orbitalIn < 0.0 ? -1.0 : 1.0) : 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update target based on whether the robot is "in-field" (between the hubs)
    var pose = driveTrain.getPose();
    var transl = pose.getTranslation();
    double x = transl.getX();
    double y = transl.getY();

    // If robot is left of the field (< 4.6) -> target blue hub. If right of field
    // (> 11.9) -> target red hub.
    if (x < 4.6) {
      target = BLUE_HUB_LOC;
      mode = Mode.HUB;
      SmartDashboard.putString("Polar/Target", "BLUE_HUB");
    } else if (x > 11.9) {
      target = RED_HUB_LOC;
      mode = Mode.HUB;
      SmartDashboard.putString("Polar/Target", "RED_HUB");
    } else {
      // Robot is in-field between the hubs; pick a volley location based on alliance
      // and Y
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        if (y > 4.0) {
          target = BLUE_LEFT_VOLLY_LOC;
          mode = Mode.VOLLEY;
          SmartDashboard.putString("Polar/Target", "BLUE_LEFT_VOLLEY");
        } else {
          target = BLUE_RIGHT_VOLLY_LOC;
          mode = Mode.VOLLEY;
          SmartDashboard.putString("Polar/Target", "BLUE_RIGHT_VOLLEY");
        }
      } else {
        // For red alliance choose the volley side according to the requested mapping
        if (y < 4.0) {
          target = RED_RIGHT_VOLLY_LOC;
          mode = Mode.VOLLEY;
          SmartDashboard.putString("Polar/Target", "RED_RIGHT_VOLLEY");
        } else {
          target = RED_LEFT_VOLLY_LOC;
          mode = Mode.VOLLEY;
          SmartDashboard.putString("Polar/Target", "RED_LEFT_VOLLEY");
        }
      }
    }

  }

  public class PolarUtils {

    public static ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, Translation2d target, double radialIn,
        double orbitalIn, double MaxSpeed, double MaxAngularRate, double leadSeconds) {

      Translation2d pose = estPose.getTranslation();

      double radialInput = -radialIn;
      double orbitInput = -orbitalIn;

      // Compute vector from robot to the target (field frame)
      double dx = target.getX() - pose.getX();
      double dy = target.getY() - pose.getY();
      double r = Math.hypot(dx, dy);

      r = getRadiusFrom(pose, target);

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

      // Predict where the robot will be after `leadSeconds` using the commanded
      // field-relative velocities (vxField, vyField). This lets us aim at the
      // future robot position (lead) so shots are accurate while orbiting.
      double predictedX = pose.getX() + vxField * leadSeconds;
      double predictedY = pose.getY() + vyField * leadSeconds;

      // To keep the robot pointed at the predicted position relative to the
      // fixed target, compute the desired angle to the target from the
      // predicted robot position.
      double desiredAngle = Math.atan2(target.getY() - predictedY, target.getX() - predictedX);
      double currentAngle = estPose.getRotation().getRadians();
      // Normalize angle error to [-pi, pi]
      double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));

      // Use a P-controller plus a small feedforward term derived from the
      // commanded tangential speed to account for rotational rate while
      // orbiting: phi_dot ≈ -vTangential / r. Tune kP and kFF as needed.
      double kP = 6.0;
      double kFF = 0.9;
      double ff = 0.0;
      if (Math.abs(r) > kEpsilon) {
        ff = -vTangential / r * kFF;
      }

      double omega = kP * angleError + ff;

      // Clamp angular rate to configured maximum
      omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));

      // Convert field-relative velocities to robot-relative chassis speeds
      return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omega, estPose.getRotation());
    }

    public static double getRadiusFrom(Translation2d pose, Translation2d target) {
      return pose.getDistance(target);
    }

    public static double getEstHoodPosFrmR(double r) {
      double pos = -0.00447476 * Math.pow(r, 2)
          + 0.102495 * r
          + (-0.080651 + Constants.ShooterConstants.kHoodMinPosition);
      SmartDashboard.putNumber("Calc Hood Pos", pos);
      return pos;
    }

    public static double getEstShootVelFrmR(double r) {
      double rpm = 20.00902 * Math.pow(r, 2)
          + -2.92682 * r
          + 2625;
      SmartDashboard.putNumber("Calc Shoot V", rpm);
      return rpm;
    }
  }
}
