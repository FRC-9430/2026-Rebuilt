// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

    final SparkFlex m_intakeMotor;
    final SparkFlex m_hopperMotor;

    final SparkClosedLoopController m_intakeController;
    final RelativeEncoder m_intakeEncoder;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        this(new SparkFlex(CANConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless),
                new SparkFlex(CANConstants.HOPPER_MOTOR_CAN_ID, MotorType.kBrushless));
    }

    // Overloading constructor to make testing easier
    IntakeSubsystem(SparkFlex intakeMotor, SparkFlex hopperMotor) {
        m_intakeMotor = intakeMotor;
        m_hopperMotor = hopperMotor;

        m_intakeMotor.configure(INTAKE_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_hopperMotor.configure(HOPPER_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakeController = m_intakeMotor.getClosedLoopController();
        m_intakeEncoder = m_intakeMotor.getEncoder();
    }

    /**
     * Runs the intake at the default speed
     */
    public void setIntake() {
        m_intakeController.setSetpoint(kDefaultIntakeSpeed, ControlType.kVelocity);
    }

    /**
     * Runs the intake at a specified speed
     *
     * @param speed The speed to run the intake at
     */
    public void setIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Target the intake at a specified RPM
     *
     * @param RPM The RPM to run the intake at
     */
    public void setIntakeRPM(double RPM) {
        m_intakeController.setSetpoint(RPM, ControlType.kVelocity);
    }

    /**
     * Stops the intake
     */
    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    /**
     * Gets the intake RPM
     */
    public double getIntakeV() {
        return m_intakeEncoder.getVelocity();
    }

    /**
     * Runs the hopper motor at a specified speed
     *
     * @param speed The speed to run the hopper motor at
     */
    public void setHopper(double speed) {
        m_hopperMotor.set(speed);
    }

    /**
     * Stops the hopper motor
     */
    public void stopHopper() {
        m_hopperMotor.stopMotor();
    }

    /**
     * Stops both the intake and conveyor
     */
    public void stopAll() {
        m_intakeMotor.stopMotor();
        m_hopperMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake V", m_intakeEncoder.getVelocity());
    }

    @Override
    public void close() {
        m_intakeMotor.close();
        m_hopperMotor.close();
    }

}
