package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
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

        public static final int HOOD_ARTICULATE_CAN_ID = 20;
        public static final int FEEDER_CAN_ID = 21;
        public static final int R_SHOOTER_CAN_ID = 22;
        public static final int L_TOP_SHOOTER_CAN_ID = 23;
        public static final int L_BOT_SHOOTER_CAN_ID = 24;

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

    public static final class ShooterConstants {

        // Shooter PID
        public static final double kShooterP = 4.5E-4;
        public static final double kShooterI = 1E-10;
        public static final double kShooterD = 0.0;
        public static final double kShooterS = 0.2;
        public static final double kShooterV = 0.0024;
        public static final double kShooterA = 0.0;

        // Hood PID TODO tune hood
        public static final double kHoodP = 0.0001;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;

        // Feed PID TODO tune feeder
        public static final double kFeedP = 0.1;
        public static final double kFeedI = 0.0;
        public static final double kFeedD = 0.0;

        // Setpoints
        public static final double kShooterIdleRPM = 1000.0;
        public static final double kShooterToleranceRPM = 200.0;

        // Hood limits
        public static final double kHoodMinPosition = 0.0;
        public static final double kHoodMaxPosition = 0.8;
        public static final double kHoodPositionTolerance = 0.1;
        public static final double kHoodStowedPosition = 0.0;

        public static final double kHoodPosHoldPercent = 0.04;

        public static final SparkFlexConfig MAIN_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            MAIN_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            MAIN_SHOOTER_CONFIG.inverted(false);
            MAIN_SHOOTER_CONFIG.closedLoop.pid(kShooterP, kShooterI, kShooterD);
            MAIN_SHOOTER_CONFIG.closedLoop.outputRange(0, 1); // No moving backwards
            MAIN_SHOOTER_CONFIG.closedLoop.feedForward.sva(kShooterS, kShooterV, kShooterA);
        }

        public static final SparkFlexConfig AUX_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            AUX_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            AUX_SHOOTER_CONFIG.follow(CANConstants.R_SHOOTER_CAN_ID, true);
        }

        public static final SparkFlexConfig HOOD_CONFIG = new SparkFlexConfig();
        static {
            HOOD_CONFIG.apply(new ClosedLoopConfig().pid(kHoodP, kHoodI, kHoodD));
            HOOD_CONFIG.idleMode(IdleMode.kBrake);
            HOOD_CONFIG.inverted(false);
        }

        public static final SparkFlexConfig FEED_CONFIG = new SparkFlexConfig();
        static {
            FEED_CONFIG.apply(new ClosedLoopConfig().pid(kFeedP, kFeedI, kFeedD));
            FEED_CONFIG.idleMode(IdleMode.kBrake);
            FEED_CONFIG.inverted(false);
        }

    }

}
