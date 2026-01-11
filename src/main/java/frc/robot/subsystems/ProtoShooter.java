package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (mainSpeed > 1.0) {
            mainSpeed = 1.0;
        } else if (mainSpeed < 0.0) {
            mainSpeed = 0.0;
        }
        SmartDashboard.putNumber("MainSpeed", mainSpeed);
    }

    public void setSubSpeed(double speed) {
        subSpeed = speed;
        if (subSpeed > 1.0) {
            subSpeed = 1.0;
        } else if (subSpeed < 0.0) {
            subSpeed = 0.0;
        }
        SmartDashboard.putNumber("SubSpeed", subSpeed);
    }

}
