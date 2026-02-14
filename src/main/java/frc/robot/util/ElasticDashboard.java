package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase {

    SendableChooser<Pose2d> initialPoseChooser = new SendableChooser<Pose2d>();

    public ElasticDashboard() {

        initialPoseChooser.setDefaultOption("Blue Left", new Pose2d());
        initialPoseChooser.addOption("Blue Middle", new Pose2d());
        initialPoseChooser.addOption("Blue Right", new Pose2d());
        initialPoseChooser.addOption("Red Left", new Pose2d());
        initialPoseChooser.addOption("Red Middle", new Pose2d());
        initialPoseChooser.addOption("Red Right", new Pose2d());

        SmartDashboard.putData("InitialPose", initialPoseChooser);

    }

    public Pose2d getInitialPose() {
        return initialPoseChooser.getSelected();
    }

    @Override
    public void periodic() {

    }
}
