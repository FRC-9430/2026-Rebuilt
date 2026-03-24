package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

    private final SparkFlex m_RightTopShooterMotor;
    private final SparkFlex m_RightBottomShooterMotor;
    private final SparkFlex m_LeftTopShoooterMotor;
    private final SparkFlex m_LeftBotShoooterMotor;

    private final TalonFX m_hoodMotor;

    private final PositionVoltage HoodPV = new PositionVoltage(0).withSlot(0);
    private Double hoodSetPoint = null;

    private final TalonFX m_rightFeedMotor;
    private final TalonFX m_leftFeedMotor;

    private final VelocityVoltage FeedVV = new VelocityVoltage(0).withSlot(0);

    private final SparkFlex m_conveyorMotor;

    private final RelativeEncoder m_shooterEncoder;
    private final RelativeEncoder m_conveyorEncoder;
    private final AbsoluteEncoder m_hoodEncoder;

    private final SparkClosedLoopController m_shooterController;
    private final SparkClosedLoopController m_conveyorController;

    private final PIDController m_hoodController;
    private final SimpleMotorFeedforward m_hoodFFController;

    /**
     * Creates a new ShooterSubsystem and configures motors, encoders, and
     * closed-loop controllers.
     */
    public ShooterSubsystem() {
        m_RightTopShooterMotor = new SparkFlex(CANConstants.RIGHT_TOP_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_RightBottomShooterMotor = new SparkFlex(CANConstants.RIGHT_BOTTOM_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_LeftTopShoooterMotor = new SparkFlex(CANConstants.LEFT_TOP_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_LeftBotShoooterMotor = new SparkFlex(CANConstants.LEFT_BOTTOM_SHOOT_MOTOR_CAN_ID, MotorType.kBrushless);

        m_hoodMotor = new TalonFX(CANConstants.HOOD_MOTOR_CAN_ID);

        m_rightFeedMotor = new TalonFX(CANConstants.RIGHT_FEEDER_MOTOR_CAN_ID);
        m_leftFeedMotor = new TalonFX(CANConstants.LEFT_FEEDER_MOTOR_CAN_ID);

        m_conveyorMotor = new SparkFlex(CANConstants.CONVEYOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_RightTopShooterMotor.configure(MAIN_SHOOTER_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_RightBottomShooterMotor.configure(AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_LeftTopShoooterMotor.configure(AUX_INVERTED_MOTOR_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_LeftBotShoooterMotor.configure(AUX_INVERTED_MOTOR_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_hoodMotor.getConfigurator().apply(HOOD_MOTOR_CONFIG);

        m_rightFeedMotor.getConfigurator().apply(MAIN_FEEDER_MOTOR_CONFIG);

        m_leftFeedMotor.getConfigurator().apply(AUX_FEEDER_MOTOR_CONFIG);
        m_leftFeedMotor.setControl(new Follower(CANConstants.RIGHT_FEEDER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

        m_conveyorMotor.configure(CONVEYOR_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_shooterEncoder = m_RightTopShooterMotor.getEncoder();
        m_conveyorEncoder = m_conveyorMotor.getEncoder();
        m_hoodEncoder = m_LeftTopShoooterMotor.getAbsoluteEncoder();

        m_shooterController = m_RightTopShooterMotor.getClosedLoopController();
        m_conveyorController = m_conveyorMotor.getClosedLoopController();
        m_hoodController = new PIDController(kHoodP, kHoodI, kHoodD);
        m_hoodController.setTolerance(kHoodPositionTolerance);

        m_hoodFFController = new SimpleMotorFeedforward(kHoodS, kHoodV, kHoodA);

        m_hoodController.setSetpoint(m_hoodEncoder.getPosition());

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
        m_RightTopShooterMotor.stopMotor();
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
        m_rightFeedMotor.setControl(FeedVV.withVelocity(kDefaultFeederSpeed));
    }

    /**
     * Set the feeder closed-loop target in RPM.
     *
     * @param rps target feeder RPM
     */
    public void setFeederRPS(double rps) {
        m_rightFeedMotor.setControl(FeedVV.withVelocity(rps));
    }

    /** Stop the feeder motor. */
    public void stopFeeder() {
        m_rightFeedMotor.stopMotor();
    }

    /**
     * Get the current feeder RPM from the encoder.
     *
     * @return feeder RPM
     */
    public double getFeederRPM() {
        return m_rightFeedMotor.getVelocity().getValueAsDouble();
    }

    /** Start the conveyor at the default configured speed. */
    public void startConveyor() {
        m_conveyorController.setSetpoint(kDefaultConveyorSpeed, ControlType.kVelocity);
    }

    /**
     * 
     *
     * @param RPM motor velocity
     */
    public void setConveyorRPM(double RPM) {
        m_conveyorController.setSetpoint(RPM, ControlType.kVelocity);
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
        hoodSetPoint = position;
    }

    public void setHoodDutyCycle(double speed) {
        m_hoodMotor.set(speed);
    }

    /** Move the hood to its stowed position. */
    public void stowHood() {
        setHoodPosition(kHoodStowedPosition);
    }

    /** Stop the hood motor. */
    public void stopHood() {
        m_hoodMotor.stopMotor();
        hoodSetPoint = null;
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
     * @param targetPosition  target position to check
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
     *         * @return SparkFlex Returns main shooter motor in Shooter subsystem
     */
    public SparkFlex getMainShooterMotor() {
        return this.m_RightTopShooterMotor;
    }

    /**
     * Get secondary shooter motor with auxillary shooter configurations
     * There are two secondary motors, and a parameter is required.
     *
     *
     * @author Brady Bontrager, bbontrager
     *         * @param followerMotorType Specify which follower motor to get.
     *         * * 1 = Get Left-Top shooter motor
     *         * * 2 = Get Left-Bottom shooter motor
     *         * @return SparkFlex Gets the first follower shooter motor with aux
     *         config.
     */
    public SparkFlex getFollowerShooterMotor(int followerMotorType) {

        try {
            if (followerMotorType == 1) {
                return this.m_LeftTopShoooterMotor;
            } else if (followerMotorType == 2) {
                return this.m_LeftBotShoooterMotor;
            } else {
                throw new java.lang.IllegalArgumentException(
                        "Invalid parameter passed for shooter follower motor get function");
            }
        } catch (Exception e) {
            System.err.println(e.getMessage());
            System.err.println(e.getCause());
            System.err.println(e.getStackTrace());
            return null;
        }
    }

    // /**
    // * Get hood motor from shooter subsystem
    // *
    // * @author Brady Bontrager, bbontrager
    // * * @return SparkFlex hood motor from shooter subsystem.
    // */
    // public SparkFlex getHoodMotor() {
    // return this.m_hoodMotor;
    // }

    // /**
    // * Get feed motor from shooter subsystem
    // *
    // * @author Brady Bontrager, bbontrager
    // * * @return SparkFlex feed motor from shooter subsystem.
    // */
    // public SparkFlex getMainFeedMotor() {
    // return this.m_rightFeedMotor;
    // }

    public SparkClosedLoopController getShooterPID(String pidController) {
        // if (pidController.equals("shoot")) {
        // return m_shooterController;
        // }
        // // if (pidController.equals("feed")) {
        // // return m_feedController;
        // }
        // // if (pidController.equals("hood")) {
        // // return m_hoodController;
        // // }
        // Exception e = new java.lang.IllegalArgumentException("Illegal argument in
        // getShooterPID()");
        // System.err.println(e.getMessage());
        // System.err.println(e.getStackTrace());
        // System.err.println("Using null instead");
        return null;
    }

    /** This method is called once per scheduler run. */
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Hood V", m_hoodEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter V", m_shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Convey V", m_conveyorEncoder.getVelocity());
        SmartDashboard.putNumber("Feed V", m_rightFeedMotor.getVelocity().getValue().magnitude());

        SmartDashboard.putNumber("Hood Pos", m_hoodEncoder.getPosition());

        if (hoodSetPoint != null) {
            double in = -m_hoodController.calculate(m_hoodEncoder.getPosition(), hoodSetPoint);

            if (in > 0) {
                in = Math.min(in, 0.2);
            } else {
                in = Math.max(in, 0.0);
            }

            double FF = -m_hoodFFController.calculate(m_hoodEncoder.getVelocity());

            if (FF > 0) {
                FF = Math.min(FF, 0.15);
            } else {
                FF = Math.max(FF, 0.0);
            }

            SmartDashboard.putNumber("Hood PID Input", in);
            SmartDashboard.putNumber("Hood FF Input", FF);

            m_hoodMotor.set(in + FF);

        }

    }

    /** Stops all motors in the subsystem. */
    public void stopAll() {

        m_RightTopShooterMotor.stopMotor();
        m_hoodMotor.stopMotor();
        m_rightFeedMotor.stopMotor();
        m_conveyorMotor.stopMotor();

    }

    @Override
    public void close() {
        m_RightTopShooterMotor.close();
        m_RightBottomShooterMotor.close();
        m_LeftTopShoooterMotor.close();
        m_LeftBotShoooterMotor.close();
        m_hoodMotor.close();
        m_rightFeedMotor.close();
        // m_leftFeedMotor.close();
    }
}
