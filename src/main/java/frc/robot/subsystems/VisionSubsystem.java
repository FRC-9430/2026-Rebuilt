package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        drivetrain = drive;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    @Override
    public void periodic() {
        addVisionMeasurements();
    }

    public void addVisionMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();

        LimelightHelpers.SetRobotOrientation(
            "limelight", 
            driveState.Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate poseEstimateMT1 = getPoseEstimateMT1();

        SmartDashboard.putNumber("Robot Pose Cam Est X", poseEstimateMT1.pose.getX());
        SmartDashboard.putNumber("Robot Pose Cam Est Y", poseEstimateMT1.pose.getY());


        boolean doRejectUpdate = false;
        if (poseEstimateMT1.tagCount == 1 && poseEstimateMT1.rawFiducials.length == 1) {
            if (poseEstimateMT1.rawFiducials[0].ambiguity > .5) {
                doRejectUpdate = true;
            }
            if (poseEstimateMT1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
                
            // if (poseEstimateMT1.rawFiducials[0].ambiguity < 0.05) {
            //     drivetrain.resetPose(poseEstimateMT1.pose); // If very confident with pose: set estimated position
            // }
        }
        if (poseEstimateMT1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (poseEstimateMT1.tagCount >= 1) {
            if (poseEstimateMT1.avgTagDist > 1.5 &&
                poseEstimateMT1.avgTagDist < 4.5) {
                
                for (RawFiducial fiducial : poseEstimateMT1.rawFiducials) {
                    if (fiducial.ambiguity < 0.05) {
                        // If very confident with pose: set estimated position
                        drivetrain.resetPose(poseEstimateMT1.pose); 
                    }
                }    
                
            }
        }

        

        if (!doRejectUpdate) {
            drivetrain.addVisionMeasurement(
                    poseEstimateMT1.pose,
                    poseEstimateMT1.timestampSeconds);
        }

    }

}
