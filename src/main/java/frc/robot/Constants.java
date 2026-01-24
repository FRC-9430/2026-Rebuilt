
package frc.robot;

/**
 * Container class for other Constants classes
 */
public final class Constants {

    /**
     * CAN IDs for all CAN devices
     */
    public static final class CANConstants {
        public static final int climberMotor1CanID = 21; //TODO: get motor can ID
        public static final int climberMotor2CanID = 22; //TODO: get motor can ID
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
  
    /**
    * Constants for the Climbing Arm Subsystem
    */
    public static final class ClimberArmConstants {
    
        public static final double kClimberMax = .48; //TODO: DummyValue
        public static final double kClimberMin = .090;//TODO: DummyValue
    }

}
