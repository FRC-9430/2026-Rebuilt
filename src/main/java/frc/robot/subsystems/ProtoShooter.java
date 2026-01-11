package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProtoShooter extends SubsystemBase {

    SparkFlex mainMotor1 = new SparkFlex(10, SparkFlex.MotorType.kBrushless);
    SparkFlex mainMotor2 = new SparkFlex(11, SparkFlex.MotorType.kBrushless);
    SparkFlex subMotor = new SparkFlex(12, SparkFlex.MotorType.kBrushless);

    public double mainSpeed = 0.0;
    public double subSpeed = 0.0;

    public ProtoShooter() {
    
    }

    public void runForward() {
        mainMotor1.set(mainSpeed);
        mainMotor2.set(mainSpeed);
        subMotor.set(subSpeed);
    }

    public void runBackward() {
        mainMotor1.set(-mainSpeed);
        mainMotor2.set(-mainSpeed);
        subMotor.set(-subSpeed);
    }

    public void stopMotors() {
        mainMotor1.stopMotor();
        mainMotor2.stopMotor();
        subMotor.stopMotor();
    }

    public void setMainSpeed(double speed) {
        mainSpeed = speed;
    }

    public void setSubSpeed(double speed) {
        subSpeed = speed;
    }

}
