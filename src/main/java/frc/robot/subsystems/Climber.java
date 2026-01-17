package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Climber extends SubsystemBase {
    private PIDController climberController = new PIDController(PIDConstants.kclimberP,PIDConstants.kclimberI,PIDConstants.kclimberD);

    private SparkMax ClimberMotor1;
    private SparkMax ClimberMotor2;
    private AbsoluteEncoder ClimbingEncoder;


public  ClimbingArmSubsystem() {

  ClimberMotor1 = new SparkMax(MotorConstants.ClimberMotor1CanID, SparkMax.MotorType.kBrushless);
  ClimberMotor2 = new SparkMax(MotorConstants.ClimberMotor2CanID, SparkMax.MotorType.kBrushless);



}








@Override
  public void periodic() {


climberController.calculate(ClimbingEncoder.getPosition(), 0.2);
  }
}