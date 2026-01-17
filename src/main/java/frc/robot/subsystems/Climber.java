package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ClimbingArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Climber extends SubsystemBase {
    private PIDController climberController = new PIDController(ClimbingArmConstants.kP,ClimbingArmConstants.kI,ClimbingArmConstants.kD);

    private SparkMax ClimberMotor1;
    private SparkMax ClimberMotor2;
    private AbsoluteEncoder ClimbingEncoder1;
    private AbsoluteEncoder ClimbingEncoder2;


public  ClimbingArmSubsystem() {

  ClimberMotor1 = new SparkMax(CANConstants.ClimberMotor1CanID, SparkMax.MotorType.kBrushless);
  ClimberMotor2 = new SparkMax(CANConstants.ClimberMotor2CanID, SparkMax.MotorType.kBrushless);
  ClimbingEncoder1 = ClimberMotor1.getAbsoluteEncoder();
  ClimbingEncoder2 = ClimberMotor2.getAbsoluteEncoder();
}
public void moveClimber (double speed) {


  
}








@Override
  public void periodic() {


climberController.calculate(ClimbingEncoder1.getPosition(), 0.2);
climberController.calculate(ClimbingEncoder2.getPosition(), 0.2);
  }
}