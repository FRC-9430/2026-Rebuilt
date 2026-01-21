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

    public  Climber() {

        climberMotor1 = new SparkFlex(CANConstants.climberMotor1CanID, SparkFlex.MotorType.kBrushless);
        climberMotor2 = new SparkFlex(CANConstants.climberMotor2CanID, SparkFlex.MotorType.kBrushless);
        climberEncoder1 = climberMotor1.getAbsoluteEncoder();
        climberEncoder2 = climberMotor2.getAbsoluteEncoder();
    }

    public void stopClimber1(double speed) {
        climberMotor1.stopMotor();
    }

    public void stopClimber2(double speed) {
        climberMotor2.stopMotor();
    }

    public void setClimberMotor1(double speed) {
        if (climberEncoder1.getPosition() > ClimberArmConstants.kClimber1Max
                && climberEncoder1.getPosition() < ClimberArmConstants.kClimber1Min) {
            climberMotor1.set(speed);
        } else {
            stopClimber1(speed);
        }

    }

    public void setClimberMotor2(double speed) {
        if (climberEncoder2.getPosition() > ClimberArmConstants.kClimber2Max
                && climberEncoder2.getPosition() < ClimberArmConstants.kClimber2Min) {
            climberMotor2.set(speed);
        } else {
            stopClimber2(speed);
        }
    }

    /*
     * public boolean isSetpointValid(double setpoint) {
     *  if (setpoint is greater than climbing arm minimum
     *      and setpoint is less than climbing arm maximum) {
     *          then the setpoint is valid
     *      }
     *  else // setpoint is not valid {
     *          then the setpoint is invalid
     *      }
     *  }
     *
     *
     *
     */
    @Override
    public void periodic() {

        /*
         * if setpoint is valid, THEN call .calculate(getPosition(), the setpoint)
         */
        climberController.calculate(climberEncoder1.getPosition(), 0.2);
        climberController.calculate(climberEncoder2.getPosition(), 0.2);
    }
}
