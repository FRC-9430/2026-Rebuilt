package frc.robot.subsystems;

import frc.robot.Constants.ClimbingArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    PIDController climberController = new PIDController(ClimbingArmConstants.kP, ClimbingArmConstants.kI,
            ClimbingArmConstants.kD);
}
