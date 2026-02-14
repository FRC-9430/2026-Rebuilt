package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    

    public VisionSubsystem() {
        
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

}
