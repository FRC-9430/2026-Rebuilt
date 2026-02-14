package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TagConstants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    
    private AprilTagFieldLayout layout;

    public VisionSubsystem() {
        // Try & catch for loading AprilTagFieldLayout
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        }

        catch (Exception e) {
            System.out.println("The AprilTag field layout could not be loaded.");
        }

        
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {

        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    

    public int[][] checkAllianceColor() {
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent()) {
            if (allianceColor.get() == Alliance.Red) {
                return TagConstants.redTags;

            }

            if (allianceColor.get() == Alliance.Blue) {
                return TagConstants.blueTags;

            }
        } else {
            System.out.println("No alliance color found at the time.");
        }
        return null;

    }

}
