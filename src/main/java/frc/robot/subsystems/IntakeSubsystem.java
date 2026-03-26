// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

    final TalonFX m_intakeMotor;
    final SparkFlex m_hopperMotor;

    final VelocityVoltage IntakeVV = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(CANConstants.INTAKE_MOTOR_CAN_ID);
        m_hopperMotor = new SparkFlex(CANConstants.HOPPER_MOTOR_CAN_ID, MotorType.kBrushless);

        m_intakeMotor.getConfigurator().apply(INTAKE_MOTOR_CONFIG);
        m_hopperMotor.configure(HOPPER_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Runs the intake at the default speed
     */
    public void startIntake() {
        m_intakeMotor.setControl(IntakeVV
                .withVelocity(DriverStation.isAutonomous() ? kDefaultAutoIntakeSpeed : kDefaultTeleIntakeSpeed));
    }

    /**
     * Target the intake at a specified RPS
     *
     * @param RPS The RPM to run the intake at
     */
    public void setIntakeRPS(double RPS) {
        m_intakeMotor.setControl(IntakeVV.withVelocity(RPS));
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
        return m_intakeMotor.getVelocity().getValueAsDouble();
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
        SmartDashboard.putNumber("Intake V", getIntakeV());
    }

    @Override
    public void close() {
        m_intakeMotor.close();
        m_hopperMotor.close();
    }

}
