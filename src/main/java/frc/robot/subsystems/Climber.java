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


public void ClimbingArmSubsystem() {

  ClimberMotor1 = new SparkMax(CANConstants.ClimberMotor1CanID, SparkMax.MotorType.kBrushless);
  ClimberMotor2 = new SparkMax(CANConstants.ClimberMotor2CanID, SparkMax.MotorType.kBrushless);
  ClimbingEncoder1 = ClimberMotor1.getAbsoluteEncoder();
  ClimbingEncoder2 = ClimberMotor2.getAbsoluteEncoder();
}
public void MoveClimber1 (double speed) {
  MoveClimber1(speed);
}
public void StopClimber1 (double speed){
ClimberMotor1.stopMotor();
}

public void MoveClimber2 (double speed) {
  MoveClimber2(speed);
}
public void StopClimber2 (double speed){
ClimberMotor2.stopMotor();
}

 public void setClimbingMotor1(double speed) {
    if (ClimbingEncoder1.getPosition() > ClimbingArmConstants.kClimber1Max
        && ClimbingEncoder1.getPosition() < ClimbingArmConstants.kClimber1Min ) {
          ClimberMotor1.set(speed);
        }else{
          StopClimber2(speed);
        }
      
}
public void setClimbingMotor2(double speed) {
    if (ClimbingEncoder1.getPosition() > ClimbingArmConstants.kClimber2Max
        && ClimbingEncoder1.getPosition() < ClimbingArmConstants.kClimber2Min ) {
          ClimberMotor2.set(speed);
        }else{
          StopClimber2(speed);
        }
}





@Override
  public void periodic() {


climberController.calculate(ClimbingEncoder1.getPosition(), 0.2);
climberController.calculate(ClimbingEncoder2.getPosition(), 0.2);
  }
}