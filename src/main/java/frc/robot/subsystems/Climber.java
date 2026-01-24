package frc.robot.subsystems;

//Import packages (thumbs up)
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants.ClimberArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

// create climber
 /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
public class Climber extends SubsystemBase {

    private SparkFlex climberMotor1;
    private SparkFlex climberMotor2;
    private AbsoluteEncoder climberEncoder1;
    private AbsoluteEncoder climberEncoder2;

    // Declare the climber
     /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
    public Climber() {
        // Declare motors and their encoders
        climberMotor1 = new SparkFlex(CANConstants.climberMotor1CanID, SparkFlex.MotorType.kBrushless);
        climberMotor2 = new SparkFlex(CANConstants.climberMotor2CanID, SparkFlex.MotorType.kBrushless);
        climberEncoder1 = climberMotor1.getAbsoluteEncoder();
        climberEncoder2 = climberMotor2.getAbsoluteEncoder();
    }

    // stop climber command
     /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
    public void stopClimber() {
        climberMotor1.stopMotor();
        climberMotor2.stopMotor();
    }

    // Verify motors are not above or below the limits
     /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
    public boolean climberMotor1Valid() {
        if (climberEncoder1.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder1.getPosition() < ClimberArmConstants.kClimberMin)
            ;
        return true;
    }

    public boolean climberMotor2Valid() {
        if (climberEncoder2.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder2.getPosition() < ClimberArmConstants.kClimberMin)
            ;
        return true;
    }

    // If both encoders valid set climber motors
     /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
    public void setClimberMotors(double speed) {
        if (climberMotor1Valid() == true
                && climberMotor2Valid() == true)
            ;
        setClimberMotors(speed);
    }

    // get encoder postion periodicly
     /**
     * Brief description of the method or class.
     * 
     * @author Your Full Name
     *  * @param param_name description of the parameter.
     *  * @return return_type description of the return value.
     */
    @Override
    public void periodic() {
        climberEncoder1.getPosition();
        climberEncoder2.getPosition();

    }
}
