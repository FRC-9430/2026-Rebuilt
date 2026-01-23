package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {

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

    public static final class TagConstants {

      // Red team constants
      public static final int[] redHubTags = {2, 3, 4, 5, 8, 9, 10, 11};
      public static final int[] redLadderTags = {15, 16};
      public static final int[] redDepotTags = {13, 14};
      public static final int[] redTrenchTags = {1, 6, 7, 12};
      int[][] redTags = {redHubTags, redLadderTags, redDepotTags, redTrenchTags};

      // Blue team constants
      public static final int[] blueHubTags = {18, 19, 20, 21, 24, 25, 26};
      public static final int[] blueLadderTags = {31, 32};
      public static final int[] blueDepotTags = {29, 30};
      public static final int[] blueTrenchTags = {17, 22, 23,28};  
      int[][] blueTags = {blueHubTags, blueLadderTags, blueDepotTags, blueTrenchTags};  
    }

  }
