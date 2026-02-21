package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex m_RightShooterMotor;
    private final SparkFlex m_LeftTopShoooterMotor;
    private final SparkFlex m_LeftBotShoooterMotor;

    private final SparkFlex m_hoodMotor;
    private final SparkFlex m_feedMotor;
    private final SparkFlex m_conveyorMotor;

    private final RelativeEncoder m_shooterEncoder;
    private final AbsoluteEncoder m_hoodEncoder;
    private final RelativeEncoder m_feedEncoder;

    private final SparkClosedLoopController m_shooterController;
    private final SparkClosedLoopController m_hoodController;
    private final SparkClosedLoopController m_feedController;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_RightShooterMotor = new SparkFlex(CANConstants.RIGHT_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_LeftTopShoooterMotor = new SparkFlex(CANConstants.LEFT_TOP_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_LeftBotShoooterMotor = new SparkFlex(CANConstants.LEFT_BOTTOM_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);

        m_hoodMotor = new SparkFlex(CANConstants.HOOD_MOTOR_CAN_ID, MotorType.kBrushless);
        m_feedMotor = new SparkFlex(CANConstants.FEEDER_MOTOR_CAN_ID, MotorType.kBrushless);
        m_conveyorMotor = new SparkFlex(CANConstants.CONVEYOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_RightShooterMotor.configure(MAIN_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_LeftTopShoooterMotor.configure(AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_LeftBotShoooterMotor.configure(AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_hoodMotor.configure(HOOD_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_feedMotor.configure(FEED_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_conveyorMotor.configure(CONVEYOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_shooterEncoder = m_RightShooterMotor.getEncoder();
        m_hoodEncoder = m_hoodMotor.getAbsoluteEncoder();
        m_feedEncoder = m_feedMotor.getEncoder();

        m_shooterController = m_RightShooterMotor.getClosedLoopController();
        m_hoodController = m_hoodMotor.getClosedLoopController();
        m_feedController = m_feedMotor.getClosedLoopController();
    }

    /**
     * Sets the target RPM for the top and bottom Shooter motors.
     *
     * @param topRPM    The target RPM for the top Shooter.
     * @param bottomRPM The target RPM for the bottom Shooter.
     */
    public void setShooterSpeedsRPM(double speed) {
        m_shooterController.setSetpoint(speed, ControlType.kVelocity);
    }

    /**
     * Sets the target speed for the top and bottom Shooter motors.
     *
     * @param topRPM    The target RPM for the top Shooter.
     * @param bottomRPM The target RPM for the bottom Shooter.
     */
    public void setShooterSpeedsPercentage(double speed) {
        m_RightShooterMotor.set(speed);
    }

    /** Sets the Shooters to a slow idle speed. */
    public void idleShooter() {
        m_shooterController.setSetpoint(kShooterIdleRPM, ControlType.kVelocity);
    }

    /** Stops the Shooter motors. */
    public void stopShooter() {
        m_RightShooterMotor.stopMotor();
    }

    /**
     * Gets the average RPM of the Shooters.
     *
     * @return The average Shooter RPM.
     */
    public double getShooterRPM() {
        return m_shooterEncoder.getVelocity();
    }

    /**
     * Checks if the Shooters are at their target speed.
     *
     * @return True if the Shooters are at speed, false otherwise.
     */
    public boolean shooterIsAtSpeed() {
        return Math.abs(getShooterRPM() - m_shooterController.getSetpoint()) <= kShooterToleranceRPM;
    }

    /**
     * Runs the feeder motor at the default speed
     */
    public void setFeeder() {
        m_feedMotor.set(kDefaultFeederSpeed);
    }

    /**
     * Runs the feeder motor at the specified RPM.
     *
     * @param RPM The speed to run the feeder motor at.
     */
    public void setFeederRPM(double RPM) {
        m_feedController.setSetpoint(RPM, ControlType.kVelocity);
    }

    /**
     * Runs the feeder motor at the specified speed.
     *
     * @param speed The speed to run the feeder motor at.
     */
    public void setFeederPercentage(double speed) {
        m_feedMotor.set(speed);
    }

    /** Stops the feeder motor. */
    public void stopFeeder() {
        m_feedMotor.stopMotor();
    }

    /**
     * Gets the current RPM of the feeder motor.
     *
     * @return The feeder RPM.
     */
    public double getFeederRPM() {
        return m_feedEncoder.getVelocity();
    }

    /**
     * Runs the conveyor at default speed
     * 
     * @param speed The speed to run the conveyor at
     */
    public void setConveyor() {
        m_conveyorMotor.set(kDefaultFeederSpeed);
    }

    /**
     * Runs the conveyor at a specified speed
     * 
     * @param speed The speed to run the conveyor at
     */
    public void setConveyor(double speed) {
        m_conveyorMotor.set(speed);
    }

    /**
     * Stops the conveyor
     */
    public void stopConveyor() {
        m_conveyorMotor.stopMotor();
    }

    /**
     * Sets the position of the shooter hood.
     *
     * @param position The target hood position.
     */
    public void setShootingAngle(double position) {
        m_hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    /** Moves the hood to its stowed position. */
    public void stowHood() {
        setShootingAngle(kHoodStowedPosition);
    }

    /**
     * Manually controls the hood motor.
     *
     * @param speed The speed to set the motor to.
     */
    public void manualHoodControl(double speed) {
        m_hoodMotor.set(speed);
    }

    /** Stops the hood motor. */
    public void stopHood() {
        m_hoodController.setSetpoint(getHoodPosition(), ControlType.kPosition);
        m_hoodMotor.stopMotor();
    }

    /**
     * Checks if the hood is at a given position.
     *
     * @param position The position to check against.
     * @return True if the hood is at the position, false otherwise.
     */
    public boolean hoodAtPosition(double position) {
        return Math.abs(getHoodPosition() - position) <= kHoodPositionTolerance;
    }

    /**
     * Checks if the hood is in its stowed position.
     *
     * @return True if the hood is stowed, false otherwise.
     */
    public boolean isHoodStowed() {
        return hoodAtPosition(kHoodStowedPosition);
    }

    /**
     * Gets the current position of the hood.
     *
     * @return The hood position.
     */
    public double getHoodPosition() {
        return m_hoodEncoder.getPosition();
    }

    /**
     * Checks if the Shooters are at their target RPM and the hood is at its target
     * position.
     *
     * @return true if the shooter is ready to fire, false otherwise.
     */
    public boolean isReadyToShoot() {
        return shooterIsAtSpeed() && hoodAtPosition(m_hoodController.getSetpoint());
    }

    /** This method is called once per scheduler run. */
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Hood V", m_hoodEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter V", m_shooterEncoder.getVelocity());

        SmartDashboard.putNumber("Hood Pos", m_hoodEncoder.getPosition());

        // When the hood is stowed, turn the motor off
        if ((m_hoodController.isAtSetpoint() || m_hoodEncoder.getPosition() <= kHoodStowedPosition)
                && m_hoodController.getSetpoint() == kHoodStowedPosition) {
        }

    }

    /** Stops all motors in the subsystem. */
    public void stopAll() {

        m_RightShooterMotor.stopMotor();
        m_hoodMotor.stopMotor();
        m_feedMotor.stopMotor();
        m_conveyorMotor.stopMotor();

    }
}
