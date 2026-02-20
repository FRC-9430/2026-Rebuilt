package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase {

    SendableChooser<Pose2d> initialPoseChooser = new SendableChooser<Pose2d>();
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public ElasticDashboard() {

        initialPoseChooser.setDefaultOption("Blue Left", new Pose2d());
        initialPoseChooser.addOption("Blue Middle", new Pose2d());
        initialPoseChooser.addOption("Blue Right", new Pose2d());
        initialPoseChooser.addOption("Red Left", new Pose2d());
        initialPoseChooser.addOption("Red Middle", new Pose2d());
        initialPoseChooser.addOption("Red Right", new Pose2d());

        SmartDashboard.putData("InitialPose", initialPoseChooser);

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public Pose2d getInitialPose() {
        return initialPoseChooser.getSelected();
    }

    public Command getAutoChooser() {
        return autoChooser.getSelected();
    }

    @Override
    public void periodic() {

    }
}
