package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase {

    SendableChooser<String> menu = new SendableChooser<String>();

    public ElasticDashboard() {

        menu.setDefaultOption("name 1", "value 1");
        menu.addOption("name 2", "value 2");
        menu.addOption("name 3", "value 3");

        SmartDashboard.putData("dropdown", menu);
        SmartDashboard.putString("tester", "hello world :)");

    }

    public String getMenuItem() {
        return menu.getSelected();
    }

    @Override
    public void periodic() {

        String tester = SmartDashboard.getString("tester", "");
        SmartDashboard.putString("mirror", tester);

        SmartDashboard.putString("selected", menu.getSelected());
    }
}
