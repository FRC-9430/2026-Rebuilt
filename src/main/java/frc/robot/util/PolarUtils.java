// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.FieldConstants.*;

/** Add your docs here. */
public class PolarUtils {

    public static ChassisSpeeds getPolarDriveSpeeds(Pose2d estPose, double radialIn, double orbitalIn, double MaxSpeed, double MaxAngularRate) {

        // Determine target point depending on alliance
        Translation2d target;
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

        // P = 25
        double omega = 25.0 * angleError;

        // Clamp angular rate to configured maximum
        omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));

        // Convert field-relative velocities to robot-relative chassis speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, -omega, estPose.getRotation());
    }

    public static double getRadiusFrom(Translation2d pose, Translation2d target) {
        return pose.getDistance(target);
    }

    public static double getEstHoodFrmR(double r){
        double pos = 0.0800597 * r + 0.300;
        SmartDashboard.putNumber("Calc Hood Pos", pos);
        return pos;
    }
}
