// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PolarUtils {

    public static ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, double radialIn, double orbitalIn, double MaxSpeed, double MaxAngularRate) {

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
                targetPointX = 4.65;
                targetPointY = 4.034;
                break;
            default:
                // Fallback to a reasonable default on unknown alliance
                targetPointX = 4.625;
                targetPointY = 4.0;
                break;
        }

        Pose2d pose = estPose;

        // Controller mapping: forward/back controls radial motion toward/away from the
        // point.
        // Left stick: negative Y is forward on this controller mapping in this project,
        // so we negate
        double radialInput = -radialIn; // +1 => forward (towards point)
        // Left stick left/right will orbit around the point
        double orbitInput = -orbitalIn;

        radialInput = (Math.abs(radialInput) > 0.05 ? radialInput : 0);
        orbitInput = (Math.abs(orbitInput) > 0.05 ? orbitInput : 0);

        // Compute vector from robot to the target (field frame)
        double dx = targetPointX - pose.getX();
        double dy = targetPointY - pose.getY();
        double r = Math.hypot(dx, dy);

        SmartDashboard.putNumber("Dist From Hub", r);

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

        double omega = 6.0 * angleError;
        // Clamp angular rate to configured maximum
        omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));

        // Convert field-relative velocities to robot-relative chassis speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, -omega, pose.getRotation());
    }

    public static double getEstHoodFrmR(double r){
        double pos = 0.0800597 * r + 0.305799;
        SmartDashboard.putNumber("Calc Hood Pos", pos);
        return pos;
    }
}
