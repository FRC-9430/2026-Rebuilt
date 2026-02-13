package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex R_shooterMotor;
    private final SparkFlex L_topShoooterMotor;
    private final SparkFlex L_botShoooterMotor;

    private final SparkFlex m_hoodMotor;
    private final SparkFlex m_feedMotor;

    private final RelativeEncoder shooterEncoder;
    private final AbsoluteEncoder hoodEncoder;
    private final RelativeEncoder feedEncoder;

    private final SparkClosedLoopController shooterController;
    private final SparkClosedLoopController hoodController;
    private final SparkClosedLoopController feedController;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        R_shooterMotor = new SparkFlex(CANConstants.R_SHOOTER_CAN_ID, MotorType.kBrushless);
        L_topShoooterMotor = new SparkFlex(CANConstants.L_TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
        L_botShoooterMotor = new SparkFlex(CANConstants.L_BOT_SHOOTER_CAN_ID, MotorType.kBrushless);

        m_hoodMotor = new SparkFlex(CANConstants.HOOD_ARTICULATE_CAN_ID, MotorType.kBrushless);
        m_feedMotor = new SparkFlex(CANConstants.FEEDER_CAN_ID, MotorType.kBrushless);

        R_shooterMotor.configure(MAIN_SHOOTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        L_topShoooterMotor.configure(AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        L_botShoooterMotor.configure(AUX_SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_hoodMotor.configure(HOOD_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_feedMotor.configure(FEED_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterEncoder = R_shooterMotor.getEncoder();
        hoodEncoder = m_hoodMotor.getAbsoluteEncoder();
        feedEncoder = m_feedMotor.getEncoder();

        shooterController = R_shooterMotor.getClosedLoopController();
        hoodController = m_hoodMotor.getClosedLoopController();
        feedController = m_feedMotor.getClosedLoopController();
    }

    /**
     * Sets the target RPM for the top and bottom Shooter motors.
     *
     * @param topRPM    The target RPM for the top Shooter.
     * @param bottomRPM The target RPM for the bottom Shooter.
     */
    public void setShooterSpeedsRPM(double speed) {
        shooterController.setSetpoint(speed, ControlType.kVelocity);
    }

    /**
     * Sets the target speed for the top and bottom Shooter motors.
     *
     * @param topRPM    The target RPM for the top Shooter.
     * @param bottomRPM The target RPM for the bottom Shooter.
     */
    public void setShooterSpeedsPercentage(double speed) {
        R_shooterMotor.set(speed);
    }

    /** Sets the Shooters to a slow idle speed. */
    public void idleShooter() {
        shooterController.setSetpoint(kShooterIdleRPM, ControlType.kVelocity);
    }

    /** Stops the Shooter motors. */
    public void stopShooter() {
        R_shooterMotor.stopMotor();
    }

    /**
     * Gets the average RPM of the Shooters.
     *
     * @return The average Shooter RPM.
     */
    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Checks if the Shooters are at their target speed.
     *
     * @return True if the Shooters are at speed, false otherwise.
     */
    public boolean shooterIsAtSpeed() {
        return Math.abs(getShooterRPM() - shooterController.getSetpoint()) <= kShooterToleranceRPM;
    }

    /**
     * Runs the feeder motor at the specified RPM.
     *
     * @param RPM The speed to run the feeder motor at.
     */
    public void runFeederRPM(double RPM) {
        feedController.setSetpoint(RPM, ControlType.kVelocity);
    }

    /**
     * Runs the feeder motor at the specified speed.
     *
     * @param speed The speed to run the feeder motor at.
     */
    public void runFeederPercentage(double speed) {
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
        return feedEncoder.getVelocity();
    }

    /**
     * Sets the position of the shooter hood.
     *
     * @param position The target hood position.
     */
    public void setShootingAngle(double position) {
        hoodController.setSetpoint(position, ControlType.kPosition);
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
        hoodController.setSetpoint(getHoodPosition(), ControlType.kPosition);
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
        return hoodEncoder.getPosition();
    }

    /**
     * Checks if the Shooters are at their target RPM and the hood is at its target
     * position.
     *
     * @return true if the shooter is ready to fire, false otherwise.
     */
    public boolean isReadyToShoot() {
        return shooterIsAtSpeed() && hoodController.isAtSetpoint();
    }

    /** This method is called once per scheduler run. */
    @Override
    public void periodic() {

        // When the hood is stowed, turn the motor off
        if ((hoodController.isAtSetpoint() || hoodEncoder.getPosition() <= kHoodStowedPosition)
                && hoodController.getSetpoint() == kHoodStowedPosition) {
            stopHood();
        }
        
    }

    /** Stops all motors in the subsystem. */
    public void stopAll() {

        shooterController.setSetpoint(0.0, ControlType.kVelocity);
        feedController.setSetpoint(0.0, ControlType.kVelocity);

        L_botShoooterMotor.stopMotor();
        L_topShoooterMotor.stopMotor();
        R_shooterMotor.stopMotor();
        m_hoodMotor.stopMotor();
        m_feedMotor.stopMotor();
    }
}
