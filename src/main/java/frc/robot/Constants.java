package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
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
        public static final int HOOD_ENCODER_CAN_ID = 39;
        public static final int HOOD_MOTOR_CAN_ID = 40;

        public static final int RIGHT_FEEDER_MOTOR_CAN_ID = 41;
        public static final int LEFT_FEEDER_MOTOR_CAN_ID = 42;

        public static final int RIGHT_TOP_SHOOT_MOTOR_CAN_ID = 43;
        public static final int RIGHT_BOTTOM_SHOOT_MOTOR_CAN_ID = 44;
        public static final int LEFT_TOP_SHOOT_MOTOR_CAN_ID = 45;
        // public static final int LEFT_BOTTOM_SHOOT_MOTOR_CAN_ID = 46;

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
        public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08) // Add a 8% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final SwerveRequest.FieldCentric slow = new SwerveRequest.FieldCentric()
                .withDeadband((MaxSpeed/3.0) * 0.08).withRotationalDeadband((MaxAngularRate) * 0.08) // Add a 8% deadband
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
        public static final double kShooterP = 0.00025;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0001;
        public static final double kShooterS = 0.2;
        public static final double kShooterV = 0.001769;
        public static final double kShooterA = 0.2;

        // Hood PID
        public static final double kHoodP = 25.8;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.5;
        public static final double kHoodS = 0.0;
        public static final double kHoodV = 0.0;
        public static final double kHoodA = 0.0;

        // Feed PID
        public static final double kFeedP = 0.0005;
        public static final double kFeedI = 0.0;
        public static final double kFeedD = 0.0;
        public static final double kFeedS = 0.0;
        public static final double kFeedV = 0.161;
        public static final double kFeedA = 0.0;

        // Conveyor PID
        public static final double kConveyorP = 0.0001;
        public static final double kConveyorI = 0.0;
        public static final double kConveyorD = 0.0;
        public static final double kConveyorS = 0.0;
        public static final double kConveyorV = 0.00205;
        public static final double kConveyorA = 0.0;

        // Setpoints
        public static final double kShooterIdleRPM = 1000.0;
        public static final double kShooterToleranceRPM = 100.0;

        public static final double kDefaultConveyorSpeed = 1000;

        public static final double kHoodStowedPosition = 0.1;
        public static final double kHoodVollyPosition = 0.3;

        public static final double kDefaultFeederSpeed = 75;

        // Hood limits
        public static final double kHoodMinPosition = 0.069;
        public static final double kHoodMinSafePosition = 0.1;
        public static final double kHoodMaxSafePosition = 0.375;
        public static final double kHoodPositionTolerance = 0.02;

        public static final SparkFlexConfig MAIN_SHOOTER_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            MAIN_SHOOTER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            MAIN_SHOOTER_MOTOR_CONFIG.inverted(true);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.pid(kShooterP, kShooterI, kShooterD);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.outputRange(0, 1); // No moving backwards
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            MAIN_SHOOTER_MOTOR_CONFIG.closedLoop.feedForward.sva(kShooterS, kShooterV, kShooterA);
            MAIN_SHOOTER_MOTOR_CONFIG.smartCurrentLimit(40,20);
        }

        // public static final SparkFlexConfig AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG = new SparkFlexConfig();
        // static {
        //     AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
        //     AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG.follow(CANConstants.RIGHT_TOP_SHOOT_MOTOR_CAN_ID);
        //     AUX_NONINVERTED_SHOOTER_MOTOR_CONFIG.smartCurrentLimit(60);
        // }

        public static final SparkFlexConfig AUX_INVERTED_MOTOR_SHOOTER_CONFIG = new SparkFlexConfig();
        static {
            AUX_INVERTED_MOTOR_SHOOTER_CONFIG.idleMode(IdleMode.kCoast);
            AUX_INVERTED_MOTOR_SHOOTER_CONFIG.follow(CANConstants.LEFT_TOP_SHOOT_MOTOR_CAN_ID, true);
            AUX_INVERTED_MOTOR_SHOOTER_CONFIG.smartCurrentLimit(40,20);
        }

        public static final TalonFXConfiguration HOOD_MOTOR_CONFIG = new TalonFXConfiguration();
        static {
            HOOD_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            HOOD_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            HOOD_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            HOOD_MOTOR_CONFIG.Feedback.FeedbackRemoteSensorID = CANConstants.HOOD_ENCODER_CAN_ID;
            HOOD_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
            HOOD_MOTOR_CONFIG.Slot0.kP = kHoodP;
            HOOD_MOTOR_CONFIG.Slot0.kI = kHoodI;
            HOOD_MOTOR_CONFIG.Slot0.kD = kHoodD;
            HOOD_MOTOR_CONFIG.Slot0.kS = kHoodS;
            HOOD_MOTOR_CONFIG.Slot0.kV = kHoodV;
            HOOD_MOTOR_CONFIG.Slot0.kA = kHoodA;
        }

        public static final CANcoderConfiguration HOOD_CANCODER_CONFIG = new CANcoderConfiguration();
        static {
            HOOD_CANCODER_CONFIG.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            HOOD_CANCODER_CONFIG.MagnetSensor.MagnetOffset = 0.2;
            HOOD_CANCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        public static final TalonFXConfiguration MAIN_FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();
        static {
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kP = kFeedP;
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kI = kFeedI;
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kD = kFeedD;
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kS = kFeedS;
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kV = kFeedV;
            MAIN_FEEDER_MOTOR_CONFIG.Slot0.kA = kFeedA;
            MAIN_FEEDER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            MAIN_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;
            MAIN_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
            MAIN_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 20;
            
        }

        public static final TalonFXConfiguration AUX_FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();
        static {
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kP = kFeedP;
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kI = kFeedI;
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kD = kFeedD;
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kS = kFeedS;
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kV = kFeedV;
            AUX_FEEDER_MOTOR_CONFIG.Slot0.kA = kFeedA;
            AUX_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;
            AUX_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 20;
            AUX_FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        public static final SparkFlexConfig CONVEYOR_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            CONVEYOR_MOTOR_CONFIG.inverted(false);
            CONVEYOR_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            CONVEYOR_MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            CONVEYOR_MOTOR_CONFIG.closedLoop.outputRange(0, 1);
            CONVEYOR_MOTOR_CONFIG.closedLoop.pid(kConveyorP, kConveyorI, kConveyorD);
            CONVEYOR_MOTOR_CONFIG.closedLoop.feedForward.sva(kConveyorS, kConveyorV, kConveyorA);
            CONVEYOR_MOTOR_CONFIG.smartCurrentLimit(40, 20);
        }

    }

    public static final class IntakeConstants {

        public static final double kDefaultAutoIntakeSpeed = 83;
        public static final double kDefaultTeleIntakeSpeed = 75;

        public static final double kIntakeP = 0.0;
        public static final double kIntakeI = 0.0;
        public static final double kIntakeD = 0.0;
        public static final double kIntakeS = 0.0;
        public static final double kIntakeV = 0.13;
        public static final double kIntakeA = 0.0;

        public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
        static {
            INTAKE_MOTOR_CONFIG.Slot0.kP = kIntakeP;
            INTAKE_MOTOR_CONFIG.Slot0.kI = kIntakeI;
            INTAKE_MOTOR_CONFIG.Slot0.kD = kIntakeD;
            INTAKE_MOTOR_CONFIG.Slot0.kS = kIntakeS;
            INTAKE_MOTOR_CONFIG.Slot0.kV = kIntakeV;
            INTAKE_MOTOR_CONFIG.Slot0.kA = kIntakeA;
        }

        public static final SparkFlexConfig HOPPER_MOTOR_CONFIG = new SparkFlexConfig();
        static {
            HOPPER_MOTOR_CONFIG.inverted(true);
            HOPPER_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            HOPPER_MOTOR_CONFIG.smartCurrentLimit(60);
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
