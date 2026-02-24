package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.ClimberArmConstants.*;

/**
 * Climbing Arm Subsystem
 */
public class Climber extends SubsystemBase {

    private final SparkFlex leftClimberMotor;
    private final SparkFlex rightClimberMotor;

    /**
     * Constructs the climber with its motors
     * 
     * @author Amaya Lewis
     */
    public Climber() {
        // Declare motors
        leftClimberMotor = new SparkFlex(CANConstants.LEFT_CLIMB_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
        rightClimberMotor = new SparkFlex(CANConstants.RIGHT_CLIMB_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);

        // Configure Motors
        leftClimberMotor.configure(kLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightClimberMotor.configure(kRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Stops both climber motors simultaneously.
     * 
     * @author Amaya Lewis
     */
    public void stopClimbers() {
        leftClimberMotor.stopMotor();
        rightClimberMotor.stopMotor();
    }

    /**
     * Sets the climbing motor speeds
     * 
     * @author Amaya Lewis
     */
    public void setClimberMotors(double leftSpeed, double rightSpeed) {
        leftClimberMotor.set(leftSpeed);
        rightClimberMotor.set(rightSpeed);
    }

    public void setLeft(double speed) {
        leftClimberMotor.set(speed);
    }

    public void stopLeft() {
        leftClimberMotor.stopMotor();
    }
    
    public void setRight(double speed) {
        rightClimberMotor.set(speed);
    }

    public void stopRight() {
        rightClimberMotor.stopMotor();
    }

    @Override
    public void periodic() {
    }
}
