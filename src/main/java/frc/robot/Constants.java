package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Container class for other Constants classes
 */
public final class Constants {

    /**
     * CAN IDs for all CAN devices
     */
    public static final class CANConstants {
        // Essentials
        public static final int ROBORIO_CAN_ID = 0;
        public static final int PIGEON2_CAN_ID = 1;
        public static final int REV_PDH_CAN_ID = 2;

        // Swerve Module Encoders
        public static final int BR_SWERVE_ENCODER_CAN_ID = 6;
        public static final int BL_SWERVE_ENCODER_CAN_ID = 7;
        public static final int FR_SWERVE_ENCODER_CAN_ID = 8;
        public static final int FL_SWERVE_ENCODER_CAN_ID = 9;

        // Swerve Module Turning Motors
        public static final int BR_SWERVE_TURNING_CAN_ID = 10;
        public static final int BL_SWERVE_TURNING_CAN_ID = 11;
        public static final int FR_SWERVE_TURNING_CAN_ID = 12;
        public static final int FL_SWERVE_TURNING_CAN_ID = 13;

        // Swerve Module Driving Motors
        public static final int BR_SWERVE_DRIVING_CAN_ID = 14;
        public static final int BL_SWERVE_DRIVING_CAN_ID = 15;
        public static final int FR_SWERVE_DRIVING_CAN_ID = 16;
        public static final int FL_SWERVE_DRIVING_CAN_ID = 17;

        // Shooter Subsystem Motors
        public static final int HOOD_MOTOR_CAN_ID = 20;
        public static final int FEEDER_MOTOR_CAN_ID = 21;
        public static final int RIGHT_SHOOT_MOTOR_CAN_ID = 22;
        public static final int LEFT_TOP_SHOOT_MOTOR_CAN_ID = 23;
        public static final int LEFT_BOTTOM_SHOOT_MOTOR_CAN_ID = 24;
        public static final int CONVEYOR_MOTOR_CAN_ID = 26;

        // Intake Subsystem Motors
        public static final int INTAKE_MOTOR_CAN_ID = 27;
        public static final int HOPPER_MOTOR_CAN_ID = 28;

        // Climbing Motors
        public static final int LEFT_CLIMB_MOTOR_CAN_ID = 31;
        public static final int RIGHT_CLIMB_MOTOR_CAN_ID = 32;

    }

    public static final class DriveConstants {
        // Port 0 - default port
        public static final int kControllerPort = 0;

        // kSpeedAt12Volts desired top speed
        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        // 3/4 of a rotation per second max angular velocity
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        public static final SwerveRequest.ApplyRobotSpeeds aim = new SwerveRequest.ApplyRobotSpeeds();

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
        public static final double kShooterP = 0.0;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterS = 0.0;
        public static final double kShooterV = 0.001888;
        public static final double kShooterA = 0.0;

        // Hood PID
        public static final double kHoodP = 25.0;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;

        // Feed PID TODO tune feeder
        public static final double kFeedP = 0.1;
        public static final double kFeedI = 0.0;
        public static final double kFeedD = 0.0;

        // Setpoints
        public static final double kShooterIdleRPM = 1000.0;
        public static final double kShooterToleranceRPM = 100.0;

        public static final double kDefaultConveyorSpeed = 0.2;

        public static final double kHoodStowedPosition = 0.350;

        public static final double kDefaultFeederSpeed = 0.5;

        // Hood limits
        public static final double kHoodMinPosition = 0.3433;
        public static final double kHoodMinSafePosition = 0.350;
        public static final double kHoodMaxSafePosition = 0.875;
        public static final double kHoodPositionTolerance = 0.1;

        public static final SparkFlexConfig MAIN_SHOOTER_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            MAIN_SHOOTER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            MAIN_SHOOTER_MOTOR_CONFIG.inverted(false);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.pid(kShooterP, kShooterI, kShooterD);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.outputRange(0, 1); // No moving backwards
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.feedForward.sva(kShooterS, kShooterV, kShooterA);
        }

        public static final SparkFlexConfig AUX_MOTOR_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            AUX_MOTOR_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            AUX_MOTOR_SHOOTER_CONFIG.follow(CANConstants.RIGHT_SHOOT_MOTOR_CAN_ID, true);
        }

        public static final SparkFlexConfig HOOD_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            HOOD_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            HOOD_MOTOR_CONFIG.inverted(false);
            HOOD_MOTOR_CONFIG.closedLoop.outputRange(-0.05, 1.0);
            HOOD_MOTOR_CONFIG.closedLoop.pid(kHoodP, kHoodI, kHoodD);
            HOOD_MOTOR_CONFIG.closedLoop.positionWrappingEnabled(false);
            HOOD_MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            HOOD_MOTOR_CONFIG.closedLoop.maxMotion.cruiseVelocity(1000);
            HOOD_MOTOR_CONFIG.closedLoop.maxMotion.maxAcceleration(10000);
            HOOD_MOTOR_CONFIG.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        }

        public static final SparkFlexConfig FEEDER_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            FEEDER_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            FEEDER_MOTOR_CONFIG.inverted(false);
            FEEDER_MOTOR_CONFIG.closedLoop.pid(kFeedP, kFeedI, kFeedD);
        }

        public static final SparkFlexConfig CONVEYOR_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            CONVEYOR_MOTOR_CONFIG.inverted(true);
            CONVEYOR_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
        }

    }

    public static final class IntakeConstants {

        public static final double kDefaultIntakeSpeed = 2500;

        public static final double kIntakeP = 0.0;
        public static final double kIntakeI = 0.0;
        public static final double kIntakeD = 0.0;
        public static final double kIntakeS = 0.0;
        public static final double kIntakeV = 0.001888;
        public static final double kIntakeA = 0.0;

        public static final SparkFlexConfig INTAKE_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            INTAKE_MOTOR_CONFIG.inverted(false);
            INTAKE_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            INTAKE_MOTOR_CONFIG.closedLoop.pid(kIntakeP, kIntakeI, kIntakeD);
            INTAKE_MOTOR_CONFIG.closedLoop.outputRange(0, 1); // No moving backwards
            INTAKE_MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            INTAKE_MOTOR_CONFIG.closedLoop.feedForward.sva(kIntakeS, kIntakeV, kIntakeA);
        }

        public static final SparkFlexConfig HOPPER_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            HOPPER_MOTOR_CONFIG.inverted(false);
            HOPPER_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
        }

    }

    public static final class FieldConstants {
        public static final Translation2d BLUE_HUB_LOC = new Translation2d(4.6525, 4.034);
        public static final Translation2d RED_HUB_LOC = new Translation2d(11.915, 4.034);

        public static final Translation2d BLUE_LEFT_VOLLY_LOC = new Translation2d(3.386, 6.250);
        public static final Translation2d BLUE_RIGHT_VOLLY_LOC = new Translation2d(3.386, 2.375);

        public static final Translation2d RED_LEFT_VOLLY_LOC = new Translation2d(13.65, 6.250);
        public static final Translation2d RED_RIGHT_VOLLY_LOC = new Translation2d(13.65, 2.375);

    }

    /**
    * Constants for the Climbing Arm Subsystem
    */
    public static final class ClimberArmConstants {

        // TODO: Tune Values
        public static final double kLeftP = 0.0001;
        public static final double kLeftI = 0;
        public static final double kLeftD = 0;
        public static final double kLeftS = 0.2;
        public static final double kLeftV = 0.001745;
        public static final double kLeftA = 0;

        public static final double kRightP = 0.0001;
        public static final double kRightI = 0;
        public static final double kRightD = 0;
        public static final double kRightS = 0.2;
        public static final double kRightV = 0.001692;
        public static final double kRightA = 0;

        public static final double kMaxSpeed = 0.0; // TODO calibrate
        public static final double kTargetRPM = 700.0;

        public static final SparkFlexConfig LEFT_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            LEFT_MOTOR_CONFIG.inverted(true);
            LEFT_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            LEFT_MOTOR_CONFIG.closedLoop.pid(kLeftP, kLeftI, kLeftD);
            LEFT_MOTOR_CONFIG.closedLoop.feedForward.sva(kLeftS, kLeftV, kLeftA);
        }

        public static final SparkFlexConfig RIGHT_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            RIGHT_MOTOR_CONFIG.inverted(false);
            RIGHT_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            RIGHT_MOTOR_CONFIG.closedLoop.pid(kRightP, kRightI, kRightD);
            RIGHT_MOTOR_CONFIG.closedLoop.feedForward.sva(kRightS, kRightV, kRightA);
        }

    }

}
