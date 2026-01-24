package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.ClimberArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Climber extends SubsystemBase {


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

    public void stopClimber() {
        climberMotor1.stopMotor();
        climberMotor2.stopMotor();
    }


    public boolean climberMotor1Valid() {
        if (climberEncoder1.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder1.getPosition() < ClimberArmConstants.kClimberMin);
            return true;
    }

    public boolean climberMotor2Valid() {
        if (climberEncoder2.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder2.getPosition() < ClimberArmConstants.kClimberMin);
            return true;
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
     * public void setSetpoint(setpoint) {
     *  if the setpoint is valid,
     *      then this.setpoint = setpoint // keep in mind, this.anything requires a pre-defined instance variable
     * }
     *
     */
    @Override
    public void periodic() {
        climberEncoder1.getPosition();
        climberEncoder2.getPosition();

        
    }
}
