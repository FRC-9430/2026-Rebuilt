package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ClimbingArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private PIDController climberController = new PIDController(ClimbingArmConstants.kclimberP,ClimbingArmConstants.kclimberI,ClimbingArmConstants.kclimberD);

    private SparkMax ClimberMotor1;
    private SparkMax ClimberMotor2;
    private AbsoluteEncoder ClimbingEncoder;
public void ClimbingArmSubsystem() {}








@Override
  public void periodic() {


climberController.calculate(ClimbingEncoder.getPosition(), 0.2);
  }
}