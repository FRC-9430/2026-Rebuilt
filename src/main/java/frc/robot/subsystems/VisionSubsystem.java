package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TagConstants;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout layout;

    public VisionSubsystem() {
        // Try & catch for loading AprilTagFieldLayout
        try{
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        }

        catch(Exception e) {
        System.out.println("The AprilTag field layout could not be loaded.");
        }


    }



    public void checkAllianceColor(){
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent()){
            if (allianceColor.get() == Alliance.Red){
                //write later - assign red team tags somehow??

            }

            if (allianceColor.get() == Alliance.Blue){
                //write later - same thing but blue team tags

            }
        }
        else{
            System.out.println("No alliance color found at the time.");

        }

    }

} 
