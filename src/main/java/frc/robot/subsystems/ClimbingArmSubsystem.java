package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberArmConstants;

import static frc.robot.Constants.ClimberArmConstants.*;

/**
 * Subsytem for controlling the climbing arms
 */
public class ClimbingArmSubsystem extends SubsystemBase {

    private final SparkFlex leftClimberMotor;
    private final SparkClosedLoopController lClimbController;
    private final RelativeEncoder lClimbEncoder;
    private final SparkFlex rightClimberMotor;
    private final SparkClosedLoopController rClimbController;
    private final RelativeEncoder rClimbEncoder;

    /**
     * Constructs the climber with its motors
     *
     * @author Amaya Lewis
     */
    public ClimbingArmSubsystem() {
        // Declare motors
        leftClimberMotor = new SparkFlex(CANConstants.LEFT_CLIMB_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
        rightClimberMotor = new SparkFlex(CANConstants.RIGHT_CLIMB_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);

        // Configure Motors
        leftClimberMotor.configure(kLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightClimberMotor.configure(kRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lClimbController = leftClimberMotor.getClosedLoopController();
        lClimbEncoder = leftClimberMotor.getEncoder();
        rClimbController = rightClimberMotor.getClosedLoopController();
        rClimbEncoder = rightClimberMotor.getEncoder();

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
     * Motors are inverted, so + goes one way and - goes the other
     *
     * @author Amaya Lewis
     * @param speed Non-controlled speed of motor (between -/+1.0)
     */
    public void setClimberMotors(double speed) {
        leftClimberMotor.set(speed);
        rightClimberMotor.set(speed);
    }

    /**
     * Sets the left climber to specified speed
     *
     * @param speed
     */
    public void setLeft(double speed) {
        leftClimberMotor.set(speed);
    }

    /**
     * Stops the left climber
     */
    public void stopLeft() {
        leftClimberMotor.stopMotor();
    }

    /**
     * Sets the right climber to specified speed
     *
     * @param speed
     */
    public void setRight(double speed) {
        rightClimberMotor.set(speed);
    }

    /**
     * Stops the right climber
     */
    public void stopRight() {
        rightClimberMotor.stopMotor();
    }

        /**
     * Target the intake at a specified RPM
     *
     * @param RPM The RPM to run the intake at
     */
    public void setClimberRPM(double RPM) {
        lClimbController.setSetpoint(RPM, ControlType.kVelocity);
        rClimbController.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RClimbRPM", rClimbEncoder.getVelocity());
        SmartDashboard.putNumber("LClimbRPM", lClimbEncoder.getVelocity());
    }
}
