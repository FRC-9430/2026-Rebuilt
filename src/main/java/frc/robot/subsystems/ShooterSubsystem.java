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

/**
 * Subsystem that manages shooter motors, feeder, conveyor, and hood.
 *
 * This class provides methods to control shooter RPM, feeder and conveyor
 * speeds, and the hood position. It also exposes status checks used by
 * higher-level commands.
 */
public class ShooterSubsystem extends SubsystemBase implements AutoCloseable{

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

    /**
     * Creates a new ShooterSubsystem and configures motors, encoders, and
     * closed-loop controllers.
     */
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
     * Set the target shooter RPM for closed-loop control.
     *
     * @param rpm target RPM for the shooter
     */
    public void setShooterRPM(double rpm) {
        m_shooterController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Set the shooter to a slow idle RPM (open/closed-loop as configured).
     */
    public void idleShooter() {
        m_shooterController.setSetpoint(kShooterIdleRPM, ControlType.kVelocity);
    }

    /** Stop the shooter motors immediately (open-loop stop). */
    public void stopShooter() {
        m_RightShooterMotor.stopMotor();
    }

    /**
     * Get the current shooter encoder velocity (RPM).
     *
     * @return current shooter RPM as reported by the encoder
     */
    public double getShooterRPM() {
        return m_shooterEncoder.getVelocity();
    }

    /**
     * Returns true when the shooter is within tolerance of the setpoint.
     *
     * @return true if shooter RPM is at the requested setpoint
     */
    public boolean isShooterAtSpeed() {
        return Math.abs(getShooterRPM() - m_shooterController.getSetpoint()) <= kShooterToleranceRPM;
    }

    public boolean isShooterAtSpeed(double rpm, double setpoint) {
        return Math.abs(rpm - setpoint) <= kShooterToleranceRPM;
    }

    /**
     * Start the feeder motor at the default configured speed.
     */
    public void startFeeder() {
        m_feedMotor.set(kDefaultFeederSpeed);
    }

    /**
     * Set the feeder closed-loop target in RPM.
     *
     * @param rpm target feeder RPM
     */
    public void setFeederRPM(double rpm) {
        m_feedController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Set the feeder motor output as a percent (-1.0 to 1.0).
     *
     * @param percent motor output percent
     */
    public void setFeederPercent(double percent) {
        m_feedMotor.set(percent);
    }

    /** Stop the feeder motor. */
    public void stopFeeder() {
        m_feedMotor.stopMotor();
    }

    /**
     * Get the current feeder RPM from the encoder.
     *
     * @return feeder RPM
     */
    public double getFeederRPM() {
        return m_feedEncoder.getVelocity();
    }

    /** Start the conveyor at the default configured speed. */
    public void startConveyorDefault() {
        m_conveyorMotor.set(kDefaultFeederSpeed);
    }

    /**
     * Set the conveyor motor output as a percent (-1.0 to 1.0).
     *
     * @param percent motor output percent
     */
    public void setConveyorPercent(double percent) {
        m_conveyorMotor.set(percent);
    }

    /** Stop the conveyor motor. */
    public void stopConveyor() {
        m_conveyorMotor.stopMotor();
    }

    /**
     * Set the hood target position for motion-profiled position control.
     *
     * @param position target hood position
     */
    public void setHoodPosition(double position) {
        m_hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    /** Move the hood to its stowed position. */
    public void stowHood() {
        setHoodPosition(kHoodStowedPosition);
    }

    /**
     * Manually control the hood motor output.
     *
     * @param percent motor output percent (-1.0 to 1.0)
     */
    public void manualHood(double percent) {
        m_hoodMotor.set(percent);
    }

    /** Stop the hood motor. */
    public void stopHood() {
        m_hoodMotor.stopMotor();
    }

    /**
     * Returns true when the hood is within tolerance of a target position.
     *
     * @param position target position to check
     * @return true if hood is at the given position
     */
    public boolean isHoodAtPosition(double position) {
        return isHoodAtPosition(getHoodPosition(), position);
    }

    /**
     * Returns true when the hood is within tolerance of a target position.
     * Overloaded method to allow testing with explicit values.
     *
     * @param currentPosition the current position to check against
     * @param targetPosition target position to check
     * @return true if hood is at the given position
     */
    public boolean isHoodAtPosition(double currentPosition, double targetPosition) {
        return Math.abs(currentPosition - targetPosition) <= kHoodPositionTolerance;
    }

    /** Return true when the hood is in its stowed position. */
    public boolean isHoodStowed() {
        return isHoodAtPosition(kHoodStowedPosition);
    }

    /**
     * Return true when the hood is in its stowed position.
     * Overloaded method to allow testing with explicit values.
     *
     * @param currentPosition the current position to check against
     * @return true if hood is at the stowed position
     */
    public boolean isHoodStowed(double currentPosition) {
        return isHoodAtPosition(currentPosition, kHoodStowedPosition);
    }

    /** Get the current hood encoder position. */
    public double getHoodPosition() {
        return m_hoodEncoder.getPosition();
    }

    /**
     * Returns true when the shooter RPM is at setpoint and the hood is at the
     * hood setpoint.
     *
     * @return true if the mechanism is ready to fire
     */
    public boolean isShooterReady() {
        return isShooterAtSpeed() && isHoodAtPosition(m_hoodController.getSetpoint());
    }

    /**
     * Get primary shooter motor with main shooter configurations.
     *
     * @author Brady Bontrager, bbontrager
     *  * @return SparkFlex Returns main shooter motor in Shooter subsystem
     */
    public SparkFlex getMainShooterMotor() {
        return this.m_RightShooterMotor;
    }

    /**
     * Get secondary shooter motor with auxillary shooter configurations
     * There are two secondary motors, and a parameter is required.
     *
     *
     * @author Brady Bontrager, bbontrager
     *  * @param followerMotorType Specify which follower motor to get.
     *  *   * 1 = Get Left-Top shooter motor
     *  *   * 2 = Get Left-Bottom shooter motor
     *  * @return SparkFlex Gets the first follower shooter motor with aux config.
     */
    public SparkFlex getFollowerShooterMotor(int followerMotorType) {

        try {
            if (followerMotorType == 1) {
                return this.m_LeftTopShoooterMotor;
            } else if (followerMotorType == 2) {
                return this.m_LeftBotShoooterMotor;
            }
            else {
                throw new java.lang.IllegalArgumentException("Invalid parameter passed for shooter follower motor get function");
            }
        } catch (Exception e) {
            System.err.println(e.getMessage());
            System.err.println(e.getCause());
            System.err.println(e.getStackTrace());
            return null;
        }
    }

    /**
     * Get hood motor from shooter subsystem
     *
     * @author Brady Bontrager, bbontrager
     *  * @return SparkFlex hood motor from shooter subsystem.
     */
    public SparkFlex getHoodMotor() {
        return this.m_hoodMotor;
    }

    /**
     * Get feed motor from shooter subsystem
     *
     * @author Brady Bontrager, bbontrager
     *         * @return SparkFlex feed motor from shooter subsystem.
     */
    public SparkFlex getFeedMotor() {
        return this.m_feedMotor;
    }

    public SparkClosedLoopController getShooterPID(String pidController) {
        if (pidController.equals("shoot")) {
            return m_shooterController;
        }
        if (pidController.equals("feed")) {
            return m_feedController;
        }
        if (pidController.equals("hood")) {
            return m_hoodController;
        }
        Exception e = new java.lang.IllegalArgumentException("Illegal argument in getShooterPID()");
        System.err.println(e.getMessage());
        System.err.println(e.getStackTrace());
        System.err.println("Using null instead");
        return null;
    }

    /** This method is called once per scheduler run. */
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Hood V", m_hoodEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter V", m_shooterEncoder.getVelocity());

        SmartDashboard.putNumber("Hood Pos", m_hoodEncoder.getPosition());

        // When the hood is stowed, turn the motor off
        if (isHoodStowed() && m_hoodController.getSetpoint() == kHoodStowedPosition) {
            m_hoodMotor.stopMotor();
        }

    }

    /** Stops all motors in the subsystem. */
    public void stopAll() {

        m_RightShooterMotor.stopMotor();
        m_hoodMotor.stopMotor();
        m_feedMotor.stopMotor();
        m_conveyorMotor.stopMotor();

    }

    @Override
    public void close() {
        m_RightShooterMotor.close();
        m_LeftTopShoooterMotor.close();
        m_LeftBotShoooterMotor.close();
        m_hoodMotor.close();
        m_feedMotor.close();
    }
}
