package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase {

    SendableChooser<Command> autoChooser;
    SendableChooser<Command> commandChooser;

    public ElasticDashboard() {

    }

    /**
     * Builds and published the AutoBuilder auto chooser
     */
    public void initAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Builds and publishes auto chooser of Named Commands to the dashboard
     * 
     * @param namedCommands
     */
    public void initCommandChooser(HashMap<String, Command> namedCommands) {
        commandChooser = new SendableChooser<Command>();
        commandChooser.setDefaultOption("No Command", Commands.none());

        namedCommands.forEach((name, command) -> {
            commandChooser.addOption(name, command);
        });

        SmartDashboard.putData("Command Chooser", commandChooser);
        SmartDashboard.putBoolean("Run Command", false);
    }

    /**
     * Returns the selected auto from the dashboard
     * 
     * @return selected auto
     */
    public Command getAutoChooser() {
        return autoChooser.getSelected();
    }

    /**
     * Returns the selected Named Command from the dashboard
     * 
     * @return selected command
     */
    public Command getCommandChooser() {
        return commandChooser.getSelected();
    }

    @Override
    public void periodic() {

        // Schedule Selected Named Command when Run Command button is pressed
        if (SmartDashboard.getBoolean("Run Command", false)) {

            var command = getCommandChooser();

            if (command != null && command != Commands.none()) {
                CommandScheduler.getInstance().schedule(command);
            }

        }

    }
}
