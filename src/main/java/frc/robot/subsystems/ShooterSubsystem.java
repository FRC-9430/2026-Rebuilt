package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex R_shooterMotor;
    private final SparkFlex L_topShoooterMotor;
    private final SparkFlex L_botShoooterMotor;

    private final SparkFlex m_hoodMotor;
    private final SparkFlex m_feedMotor;

    private final RelativeEncoder shooterEncoder;
    private final AbsoluteEncoder hoodEncoder;

    private final SparkClosedLoopController shooterController;
    private final SparkClosedLoopController hoodController;


    /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
        R_shooterMotor = new SparkFlex(CANConstants.R_SHOOTER_CAN_ID, MotorType.kBrushless);
        L_topShoooterMotor = new SparkFlex(CANConstants.L_TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
        L_botShoooterMotor = new SparkFlex(CANConstants.L_BOT_SHOOTER_CAN_ID, MotorType.kBrushless);

        m_hoodMotor = new SparkFlex(CANConstants.HOOD_ARTICULATE_CAN_ID, MotorType.kBrushless);
        m_feedMotor = new SparkFlex(CANConstants.FEEDER_CAN_ID, MotorType.kBrushless);

        R_shooterMotor.configure(ShooterConstants.MAIN_SHOOTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        L_topShoooterMotor.configure(ShooterConstants.AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        L_botShoooterMotor.configure(ShooterConstants.AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_hoodMotor.configure(ShooterConstants.HOOD_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_feedMotor.configure(ShooterConstants.FEED_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterEncoder = R_shooterMotor.getEncoder();
        hoodEncoder = m_hoodMotor.getAbsoluteEncoder();

        shooterController = R_shooterMotor.getClosedLoopController();
        hoodController = m_hoodMotor.getClosedLoopController();

    }

    /**
     * Sets the target RPM for the top and bottom flywheel motors.
     *
     * @param topRPM The target RPM for the top flywheel.
     * @param bottomRPM The target RPM for the bottom flywheel.
     */
    public void setFlywheelSpeeds(double speed) {
        R_shooterMotor.set(speed);
    }

    /** Sets the flywheels to a slow idle speed. */
    public void idleFlywheels() {
        // TODO: Run at idle RPM
    }

    /** Stops the flywheel motors. */
    public void stopFlywheels() {
        // TODO: Stop all flywheel motors
    }

    /**
     * Gets the average RPM of the flywheels.
     *
     * @return The average flywheel RPM.
     */
    public double getFlywheelRPM() {
        // TODO: Return average flywheel RPM
        return 0.0;
    }

    /**
     * Checks if the flywheels are at their target speed.
     *
     * @return True if the flywheels are at speed, false otherwise.
     */
    public boolean flywheelsAtSpeed() {
        // TODO: Check if all flywheels are within tolerance
        return false;
    }

    /**
     * Runs the feeder motor at the specified speed.
     *
     * @param speed The speed to run the feeder motor at.
     */
    public void runFeeder(double speed) {
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
        // TODO: Return feeder RPM
        return 0.0;
    }

    /**
     * Sets the position of the shooter hood.
     *
     * @param position The target hood position.
     */
    public void setShootingAngle(double position) {
        // TODO: Set hood position (clamped to limits)
    }

    /** Moves the hood to its stowed position. */
    public void stowHood() {
        // TODO: Move hood to stowed position
    }

    /**
     * Manually controls the hood motor.
     *
     * @param speed The speed to set the motor to.
     */
    public void manualHoodControl(double speed) {
        // TODO: Manual hood control
    }

    /** Stops the hood motor. */
    public void stopHood() {
        m_hoodMotor.stopMotor();
    }

    /**
     * Checks if the hood is at a given position.
     *
     * @param position The position to check against.
     * @return True if the hood is at the position, false otherwise.
     */
    public boolean hoodAtPosition(double position) {
        // TODO: Check if hood is at target position
        return false;
    }

    /**
     * Checks if the hood is in its stowed position.
     *
     * @return True if the hood is stowed, false otherwise.
     */
    public boolean isHoodStowed() {
        // TODO: Check if hood is stowed
        return false;
    }

    /**
     * Gets the current position of the hood.
     *
     * @return The hood position.
     */
    public double getHoodPosition() {
        // TODO: Return hood position
        return 0.0;
    }

    /**
     * Checks if the flywheels are at their target RPM and the hood is at its target
     * position.
     *
     * @return true if the shooter is ready to fire, false otherwise.
     */
    public boolean isReadyToShoot() {
        return false; // placeholder
    }

    /** This method is called once per scheduler run. */
    @Override
    public void periodic() {
    }

    /** Stops all motors in the subsystem. */
    public void stopAll() {
        L_botShoooterMotor.stopMotor();
        L_topShoooterMotor.stopMotor();
        R_shooterMotor.stopMotor();
        m_hoodMotor.stopMotor();
        m_feedMotor.stopMotor();
    }

    /**
     * Gets the top flywheel motor object. // TODO: Might be more worth it to get
     * object state
     *
     * @return The top flywheel motor.
     */
    public SparkFlex getTopFlywheelMotor() {
        return L_topShoooterMotor;
    }

    /**
     * Gets the bottom flywheel motor object. // TODO: Might be more worth it to get
     * object state
     *
     * @return The bottom flywheel motor.
     */
    public SparkFlex getBottomFlywheelMotor() {
        return L_botShoooterMotor;
    }

    /**
     * Gets the hood motor object. // TODO: Might be more worth it to get object
     * state
     *
     * @return The hood motor.
     */
    public SparkFlex getHoodMotor() {
        return m_hoodMotor;
    }
}
