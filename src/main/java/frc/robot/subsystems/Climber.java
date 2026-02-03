package frc.robot.subsystems;

//Import packages (thumbs up)
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants.ClimberArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

// create climber
public class Climber extends SubsystemBase {

    private SparkFlex climberMotor1;
    private SparkFlex climberMotor2;
    private AbsoluteEncoder climberEncoder1;
    private AbsoluteEncoder climberEncoder2;

     /**
     * Constructs the climber with its motors, encoderes, and classes.
     * 
     * @author Amaya Lewis
     *  * @return returns a climber object.
     */
    public Climber() {
        // Declare motors and their encoders
        climberMotor1 = new SparkFlex(CANConstants.climberMotor1CanID, SparkFlex.MotorType.kBrushless);
        climberMotor2 = new SparkFlex(CANConstants.climberMotor2CanID, SparkFlex.MotorType.kBrushless);
        climberEncoder1 = climberMotor1.getAbsoluteEncoder();
        climberEncoder2 = climberMotor2.getAbsoluteEncoder();
    }

    /**
     * Stops both climber motors simultaneously.
     * 
     * @author Amaya Lewis
     *  * @return void
     */
    public void stopClimber() {
        climberMotor1.stopMotor();
        climberMotor2.stopMotor();
    }
     /**
     * Checks if motors are above or below their limits using encoders.
     * 
     * @author Amaya Lewis
     *  * @returns boolean.
     */
    public boolean isClimberMotorsValid() {
        if (climberEncoder1.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder1.getPosition() < ClimberArmConstants.kClimberMin
                && climberEncoder2.getPosition() > ClimberArmConstants.kClimberMax
                && climberEncoder2.getPosition() < ClimberArmConstants.kClimberMin)
            ;
        return true;
    }

    
     /**
     * Runs motors if climber motors are valid.
     * 
     * @author Amaya Lewis
     *  * @return void.
     */
    public void setClimberMotors(double speed) {
        if (isClimberMotorsValid() == true) {
            climberMotor1.set(speed);
            climberMotor2.set(speed);
        }else{ stopClimber();}
    
    }
    @Override
    public void periodic() {
    }
}
