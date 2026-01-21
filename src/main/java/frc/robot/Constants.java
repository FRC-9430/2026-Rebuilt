package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {
  
    /**
     * CAN IDs for all CAN devices
     */
    public static final class CANConstants {
        
    }

    public static final class ClimbingArmConstants {
        public static final double kP = 0.001; // TODO: Dummy value
        public static final double kI = 0; // TODO: Dummy value
        public static final double kD = 0; // TODO: Dummy value
    }

      // TODO:CHANGE ALL OF BELOW
    public static final class ShooterConstants {
        // Motor CAN IDs - TODO: CHANGE ME
        public static final int kShooterLeftMotorId = 0;
        public static final int kShooterRightMotorId = 0;
        public static final int kShooterTopMotorId = 0;
        public static final int kShooterHoodMotorId = 0;
        
        // Flywheel PID
        public static final double kFlywheelP = 0.0001;
        public static final double kFlywheelI = 0.0;
        public static final double kFlywheelD = 0.0;
        public static final double kFlywheelFF = 0.0;

        // Hood PID
        public static final double kHoodP = 0.1;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;
        public static final double kHoodFF = 0.0;


        // Setpoints - TODO: CHANGE/REPLACE BELOW AS NECESSARY
        public static final double kShooterTargetRPM = 4000.0;
        public static final double kShooterIdleRPM = 1000.0;
        public static final double kFlywheelToleranceRPM = 100.0;

        // Hood limits
        public static final double kHoodMinPosition = 0.0;
        public static final double kHoodPositionTolerance = 0.1;
        public static final double kHoodStowedPosition = 0.0;
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

}
