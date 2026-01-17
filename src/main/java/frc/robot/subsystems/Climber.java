package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimbingArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    PIDController climberController = new PIDController(ClimbingArmConstants.kP,ClimbingArmConstants.kI,ClimbingArmConstants.kD);
    SparkFlex climbingArmMotor = new SparkFlex(CANConstants.kClimbingArmMotorID, MotorType.kBrushless);
    
    public Climber() {
        
    }

    public void setMotorSpeed(double speed) {
        climbingArmMotor.set(speed);
    }

    public void stopMotor() {
        climbingArmMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}