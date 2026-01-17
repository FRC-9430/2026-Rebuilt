package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.ClimberArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Climber extends SubsystemBase {
    private PIDController climberController = new PIDController(ClimberArmConstants.kP, ClimberArmConstants.kI,
            ClimberArmConstants.kD);

    private SparkFlex climberMotor1;
    private SparkFlex climberMotor2;
    private AbsoluteEncoder climberEncoder1;
    private AbsoluteEncoder climberEncoder2;

    public void ClimberArmSubsystem() {

        climberMotor1 = new SparkFlex(CANConstants.climberMotor1CanID, SparkFlex.MotorType.kBrushless);
        climberMotor2 = new SparkFlex(CANConstants.climberMotor2CanID, SparkFlex.MotorType.kBrushless);
        climberEncoder1 = climberMotor1.getAbsoluteEncoder();
        climberEncoder2 = climberMotor2.getAbsoluteEncoder();
    }

    public void StopClimber1(double speed) {
        climberMotor1.stopMotor();
    }

    public void StopClimber2(double speed) {
        climberMotor2.stopMotor();
    }

    public void setclimberMotor1(double speed) {
        if (climberEncoder1.getPosition() > ClimberArmConstants.kClimber1Max
                && climberEncoder1.getPosition() < ClimberArmConstants.kClimber1Min) {
            climberMotor1.set(speed);
        } else {
            StopClimber1(speed);
        }

    }

    public void setclimberMotor2(double speed) {
        if (climberEncoder2.getPosition() > ClimberArmConstants.kClimber2Max
                && climberEncoder2.getPosition() < ClimberArmConstants.kClimber2Min) {
            climberMotor2.set(speed);
        } else {
            StopClimber2(speed);
        }
    }

    @Override
    public void periodic() {

        climberController.calculate(climberEncoder1.getPosition(), 0.2);
        climberController.calculate(climberEncoder2.getPosition(), 0.2);
    }
}
