package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex m_hoodMotor;
    private final SparkFlex m_feedMotor;

    private final SparkFlex m_LtopFlywheelMotor;
    private final SparkFlex m_LbottomFlywheelMotor;
    private final SparkFlex m_RtopFlywheelMotor;


    /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
        m_LtopFlywheelMotor = new SparkFlex(CANConstants.L_TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_LbottomFlywheelMotor = new SparkFlex(CANConstants.L_BOT_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_RtopFlywheelMotor = new SparkFlex(CANConstants.R_SHOOTER_CAN_ID, MotorType.kBrushless);

        m_hoodMotor = new SparkFlex(CANConstants.HOOD_ARTICULATE_CAN_ID, MotorType.kBrushless);
        m_feedMotor = new SparkFlex(CANConstants.FEEDER_CAN_ID, MotorType.kBrushless);

        // TODO: Configure motors

        // TODO: Get encoders

        // TODO: Get PID controllers
    }

    /**
     * Sets the target RPM for the top and bottom flywheel motors.
     *
     * @param topRPM The target RPM for the top flywheel.
     * @param bottomRPM The target RPM for the bottom flywheel.
     */
    public void setFlywheelSpeeds(double topRPM, double bottomRPM) {
        // TODO: Set flywheel velocity for both motors
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
     * Checks if the flywheels are at their target speed.
     *
     * @return True if the flywheels are at speed, false otherwise.
     */
    public boolean flywheelsAtSpeed() {
        // TODO: Check if all flywheels are within tolerance
        return false;
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
        // TODO: Stop hood motor
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
        stopFlywheels();
        stopHood();
    }

    /**
     * Gets the top flywheel motor object. // TODO: Might be more worth it to get
     * object state
     *
     * @return The top flywheel motor.
     */
    public SparkFlex getTopFlywheelMotor() {
        return m_LtopFlywheelMotor;
    }

    /**
     * Gets the bottom flywheel motor object. // TODO: Might be more worth it to get
     * object state
     *
     * @return The bottom flywheel motor.
     */
    public SparkFlex getBottomFlywheelMotor() {
        return m_LbottomFlywheelMotor;
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
