package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ClimberArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Climber extends SubsystemBase {
    private PIDController climberController = new PIDController(ClimberArmConstants.kP, ClimberArmConstants.kI,
            ClimberArmConstants.kD);

    private SparkFlex ClimberMotor1;
    private SparkFlex ClimberMotor2;
    private AbsoluteEncoder ClimberEncoder1;
    private AbsoluteEncoder ClimberEncoder2;

    public void ClimberArmSubsystem() {

        ClimberMotor1 = new SparkFlex(CANConstants.ClimberMotor1CanID, SparkFlex.MotorType.kBrushless);
        ClimberMotor2 = new SparkFlex(CANConstants.ClimberMotor2CanID, SparkFlex.MotorType.kBrushless);
        ClimberEncoder1 = ClimberMotor1.getAbsoluteEncoder();
        ClimberEncoder2 = ClimberMotor2.getAbsoluteEncoder();
    }

    public void StopClimber1(double speed) {
        ClimberMotor1.stopMotor();
    }

    public void StopClimber2(double speed) {
        ClimberMotor2.stopMotor();
    }

    public void setClimberMotor1(double speed) {
        if (ClimberEncoder1.getPosition() > ClimberArmConstants.kClimber1Max
                && ClimberEncoder1.getPosition() < ClimberArmConstants.kClimber1Min) {
            ClimberMotor1.set(speed);
        } else {
            StopClimber1(speed);
        }

    }

    public void setClimberMotor2(double speed) {
        if (ClimberEncoder2.getPosition() > ClimberArmConstants.kClimber2Max
                && ClimberEncoder2.getPosition() < ClimberArmConstants.kClimber2Min) {
            ClimberMotor2.set(speed);
        } else {
            StopClimber2(speed);
        }
    }

    @Override
    public void periodic() {

        climberController.calculate(ClimberEncoder1.getPosition(), 0.2);
        climberController.calculate(ClimberEncoder2.getPosition(), 0.2);
    }
}
