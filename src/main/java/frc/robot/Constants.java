
package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Container class for other Constants classes
 */
public final class Constants {

    /**
     * CAN IDs for all CAN devices
     */
    public static final class CANConstants {
        public static final int ROBORIO_CAN_ID = 0;
        public static final int PIGEON2_CAN_ID = 1;
        public static final int REV_PDH_CAN_ID = 2;
        public static final int FL_SWERVE_ENCODER_CAN_ID = 6;
        public static final int FR_SWERVE_ENCODER_CAN_ID = 7;
        public static final int BL_SWERVE_ENCODER_CAN_ID = 8;
        public static final int BR_SWERVE_ENCODER_CAN_ID = 9;
        public static final int FL_SWERVE_TURNING_CAN_ID = 10;
        public static final int FR_SWERVE_TURNING_CAN_ID = 11;
        public static final int BL_SWERVE_TURNING_CAN_ID = 12;
        public static final int BR_SWERVE_TURNING_CAN_ID = 13;
        public static final int FL_SWERVE_DRIVING_CAN_ID = 14;
        public static final int FR_SWERVE_DRIVING_CAN_ID = 15;
        public static final int BL_SWERVE_DRIVING_CAN_ID = 16;
        public static final int BR_SWERVE_DRIVING_CAN_ID = 17;

        public static final int INTAKE_MOTOR_CAN_ID = 33;
        public static final int CONVEYOR_MOTOR_CAN_ID = 34;
    }

    /**
     * Controller related constants
     */
    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double JOYSTICK_DEADBAND = 0.08;
        public static final double TRIGGER_DEADBAND = 0.05;
    }

    public static final class IntakeConstants {
        public static final double kDefaultIntakeSpeed = 0.8;
        public static final double kDefaultConveyorSpeed = 0.8;

        public static final SparkFlexConfig kIntakeMotorConfig = new SparkFlexConfig();
        static {
            kIntakeMotorConfig.inverted(false);
            kIntakeMotorConfig.idleMode(IdleMode.kCoast);
        }

        public static final SparkFlexConfig kConveyorMotorConfig = new SparkFlexConfig();
        static {
            kConveyorMotorConfig.inverted(false);
            kConveyorMotorConfig.idleMode(IdleMode.kCoast);
        }


    }

}
