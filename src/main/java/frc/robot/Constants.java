package frc.robot;

public class Constants 
{
    public static final class DrivetrainConstants
    {
        public static final int leftmotor_port = 2;
        public static final int rightmotor_port = 9;
        public static final int leftmotor_follower_port = 3;
        public static final int rightmotor_follower_port = 4;

        public static final int controller_port = 0;
        public static final double stickDeadband = 0.1;
    }

    public static final class ManipulatorConstants {
        public static final int BOTTOM_MANIPULATOR_MOTOR_ID = 8;
        public static final int MIDDLE_MANIPULATOR_MOTOR_ID = 5;
        public static final int TOP_MANIPULATOR_MOTOR_ID = 7;

        public static final double INTAKE_SPEED = 0.6;
        public static final double HOLD_SPEED = 0.2;
        public static final double PLACE_SPEED = -0.3;
    }

    public static final class WristConstants 
    {
        public static final int WRIST_MOTOR_ID = 10; // TODO: !!!!! needs value !!!!!
        public static final int limitSwitchPort = 1;
        // PID Constants TODO: tune
        public static final double WRIST_PID_TOLERANCE = 0.1;
        public static final double WRIST_kP = 0.021;
        public static final double WRIST_kI = 0;
        public static final double WRIST_kD = 0;
        public static final double WRIST_kS = 0;
        public static final double WRIST_kG = 0.0377;
        public static final double WRIST_kV = 0;

        public static final double WRIST_SPEED_MODIFIER = 0.1; // 10%
        
        // Limits
        public static final double WRIST_LIMIT_TOP = -14;
        public static final double WRIST_LIMIT_BOTTOM = 0;
    
        // LEVEL CONSTANTS (radians)
        public static final double WRIST_PARK_ANGLE = 0;
        public static final double L3 = 0.0; // TODO: Change
        public static final double L2 = 0.0; // TODO: Change
        public static final double HUMAN_PICKUP = 0.0; // TODO: Change
        public static final double PLACE_ALGAE = 0.0; // TODO: Change
        public static final double PICKUP_ALGAE_L1 = 0.0; // TODO: Change
        public static final double PICKUP_ALGAE_L2 = 0.0; // TODO: Change
    }

    public static final class ElevatorConstants
    {
        public static final int leaderPort = 11;
        public static final int followerPort = 6;
        public static final int limitSwitchPort = 0;
        public static final double PID_TOLERANCE = 0.1;
        public static final double kP = 0.0; // TODO: Tune
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0; //TODO: Tune
        public static final double kV = 0;

        public static final double ELEVATOR_SPEED_MODIFIER = 1; // 100%
        public static final double ELEVATOR_HEIGHT = 1.1684; // in meters
        public static final double MOTOR_BOTTOM = 0;
        public static final double MOTOR_TOP = -70;
        public static final double MOTOR_ENCODER_POSITION_COEFFICENT = ELEVATOR_HEIGHT / (Math.abs(MOTOR_TOP) - Math.abs(MOTOR_BOTTOM));
    
        // LEVEL CONSTANTS (in meters)
        public static final double ELEVATOR_PARK_HEIGHT = 0.1; //where the elevator goes when idle
        public static final double L3 = -36;
        public static final double L2 = 0.0; // TODO: Change
        public static final double HUMAN_PICKUP = 0.0; // TODO: Change
        public static final double PLACE_ALGAE = 0.0; // TODO: Change
        public static final double PICKUP_ALGAE_L1 = 0.0; // TODO: Change
        public static final double PICKUP_ALGAE_L2 = 0.0; // TODO: Change
    }
}

