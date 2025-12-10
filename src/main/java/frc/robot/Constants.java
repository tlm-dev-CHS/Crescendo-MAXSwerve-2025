package frc.robot;

public class Constants {
    public static final int xboxControllerPort = 0;
    public static final int buttonBoardPort = 1;

    public static final double maxExtenderOutput = 6;
    public static final double maxRotatorOutput = 4;

    public static class CAN_IDs {
        public static final int armRotator = 25;
        public static final int armFollower = 20;

        public static final int armExtender = 14;
    }

    public static class Encoders {
        public static final double extenderPosition = 0.47;
        public static final double extenderVelocity = 1.5/60.0;

        public static final double rotatorPosition = (360.0/337.5);
        public static final double rotatorVelocity = rotatorPosition/60.0;
    }

    public static class PID {
        public static class ArmExtension {
            public static final double kP = 2;
            public static final double kI = 0;
            public static final double kD = 0;
    
            public static final double maxVelocity = 25;
            public static final double maxAcceleration = 10;
    
            public static final double tolerance = 2;
        }
    
        public static class ArmRotation {
            public static final double kP = 0.64;
            public static final double kI = 0;
            public static final double kD = 0;
    
            public static final double maxVelocity = 25;
            public static final double maxAcceleration = 10;
    
            public static final double tolerance = 4.0;
        }
    }

    public class Setpoints {
        public static class L1 {
            public static final double extenderHeight = 0;
            public static final double rotationAngle = 0;
        }

        public static class L2 {            
            public static final double extenderHeight = 0;
            public static final double rotationAngle = 0;
        }

        public static class L3 {           
            public static final double extenderHeight = 0;
            public static final double rotationAngle = 0;
        }

        public static class L4 {           
            public static final double extenderHeight = 0;
            public static final double rotationAngle = 0;
        }
    }
}