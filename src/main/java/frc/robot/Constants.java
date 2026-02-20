package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;

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
        public static final int RIGHT_SHOOTER_CAN_ID = 22;
        public static final int LEFT_TOP_SHOOTER_CAN_ID = 23;
        public static final int LEFT_BOTTOM_SHOOTER_CAN_ID = 24;

        public static final int CONVEYOR_MOTOR_CAN_ID = 26;
        public static final int INTAKE_MOTOR_CAN_ID = 27;
        public static final int BASKET_CAN_ID = 28;

    }

    public static final class ClimbingArmConstants {
        public static final double kP = 0.001; // TODO: Dummy value
        public static final double kI = 0; // TODO: Dummy value
        public static final double kD = 0; // TODO: Dummy value
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

    public static final class TagConstants {

        // Red team constants
        public static final int[] redHubTags = { 2, 3, 4, 5, 8, 9, 10, 11 };
        public static final int[] redLadderTags = { 15, 16 };
        public static final int[] redDepotTags = { 13, 14 };
        public static final int[] redTrenchTags = { 1, 6, 7, 12 };
        public static final int[][] redTags = { redHubTags, redLadderTags, redDepotTags, redTrenchTags };

        // Blue team constants
        public static final int[] blueHubTags = { 18, 19, 20, 21, 24, 25, 26 };
        public static final int[] blueLadderTags = { 31, 32 };
        public static final int[] blueDepotTags = { 29, 30 };
        public static final int[] blueTrenchTags = { 17, 22, 23, 28 };
        public static final int[][] blueTags = { blueHubTags, blueLadderTags, blueDepotTags, blueTrenchTags };
    }

    public static final class ShooterConstants {
        // Shooter PID
        public static final double kShooterP = 4.5E-5;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterS = 0.2;
        public static final double kShooterV = 0.00185;
        public static final double kShooterA = 0.0;

        // Hood PID
        public static final double kHoodP = 19.0;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;

        // Feed PID TODO tune feeder
        public static final double kFeedP = 0.1;
        public static final double kFeedI = 0.0;
        public static final double kFeedD = 0.0;

        // Setpoints
        public static final double kShooterIdleRPM = 1000.0;
        public static final double kShooterToleranceRPM = 200.0;

        public static final double kDefaultConveyorSpeed = 0.3;

        public static final double kHoodStowedPosition = 0.355;
        
        public static final double kDefaultFeederSpeed = 0.5;

        // Hood limits
        public static final double kHoodMinPosition = 0.3433;
        public static final double kHoodMinSafePosition = 0.350;
        public static final double kHoodMaxSafePosition = 0.875;
        public static final double kHoodPositionTolerance = 0.025;

        public static final SparkFlexConfig MAIN_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            MAIN_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            MAIN_SHOOTER_CONFIG.inverted(false);
            MAIN_SHOOTER_CONFIG.closedLoop.pid(kShooterP, kShooterI, kShooterD);
            MAIN_SHOOTER_CONFIG.closedLoop.outputRange(0, 1); // No moving backwards
            MAIN_SHOOTER_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            MAIN_SHOOTER_CONFIG.closedLoop.feedForward.sva(kShooterS, kShooterV, kShooterA);
        }

        public static final SparkFlexConfig AUX_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            AUX_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            AUX_SHOOTER_CONFIG.follow(CANConstants.RIGHT_SHOOTER_CAN_ID, true);
        }

        public static final SparkFlexConfig HOOD_CONFIG = new SparkFlexConfig();
        static {
            HOOD_CONFIG.idleMode(IdleMode.kBrake);
            HOOD_CONFIG.inverted(false);
            HOOD_CONFIG.closedLoop.outputRange(-0.5, 1.0); // Half speed when reversed
            HOOD_CONFIG.closedLoop.pid(kHoodP, kHoodI, kHoodD);
            HOOD_CONFIG.closedLoop.positionWrappingEnabled(false);
            HOOD_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            HOOD_CONFIG.closedLoop.maxMotion.cruiseVelocity(1000);
            HOOD_CONFIG.closedLoop.maxMotion.maxAcceleration(1000);
            HOOD_CONFIG.closedLoop.maxMotion.allowedProfileError(kHoodPositionTolerance);
            HOOD_CONFIG.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        }

        public static final SparkFlexConfig FEED_CONFIG = new SparkFlexConfig();
        static {
            FEED_CONFIG.idleMode(IdleMode.kBrake);
            FEED_CONFIG.inverted(false);
            FEED_CONFIG.closedLoop.pid(kFeedP, kFeedI, kFeedD);
        }

        public static final SparkFlexConfig CONVEYOR_CONFIG = new SparkFlexConfig();
        static {
            CONVEYOR_CONFIG.inverted(false);
            CONVEYOR_CONFIG.idleMode(IdleMode.kCoast);
        }

    }

    public static final class IntakeConstants {
        public static final double kDefaultIntakeSpeed = 2500;

        public static final double kIntakeP = 0.00006;
        public static final double kIntakeI = 0.0;
        public static final double kIntakeD = 0.0;
        public static final double kIntakeS = 0.2;
        public static final double kIntakeV = 0.00178;
        public static final double kIntakeA = 0.0;

        public static final SparkFlexConfig kIntakeMotorConfig = new SparkFlexConfig();
        static {
            kIntakeMotorConfig.inverted(false);
            kIntakeMotorConfig.idleMode(IdleMode.kCoast);
            kIntakeMotorConfig.closedLoop.pid(kIntakeP, kIntakeI, kIntakeD);
            kIntakeMotorConfig.closedLoop.outputRange(0, 1); // No moving backwards
            kIntakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            kIntakeMotorConfig.closedLoop.feedForward.sva(kIntakeS, kIntakeV, kIntakeA);
        }

        public static final SparkFlexConfig kBasketMotorConfig = new SparkFlexConfig();
        static {
            kBasketMotorConfig.inverted(false);
            kBasketMotorConfig.idleMode(IdleMode.kBrake);
        }

    }

    public static final class FieldConstants {
        public static final Translation2d BLUE_HUB_LOC = new Translation2d(4.6525, 4.034);
        public static final Translation2d RED_HUB_LOC = new Translation2d(11.915, 4.034);
    }

}
