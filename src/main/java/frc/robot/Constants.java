package frc.robot;

public class Constants {
    public static enum ScoringMode {
        SHOOTING,
        PASSING
    }

    public static final class HopperConstants {
        public static final double kHopperVoltage = 12.0; // example value
        public static final double kHopperSpeed = 1.0; // example value
    }

    public static final class IntakeConstants {
        public static final double kIntakeVoltage = 12.0; // example value
        public static final double kIntakeSpeed = 1.0; // example value
    }
    
    public static final class ShooterConstants {
        // Motor and Encoder Ports
        public static final int kFlywheelMotorPort = 4; // PWM port
        public static final int kHoodMotorPort = 5; // PWM port
        public static final int kHoodEncoderPort = 0; // DIO port
        
        // Shooter Parameters
        public static final double kShootVoltage = 12.0;
        public static final double kShootingRPS = 100.0; // Full speed for shooting
        public static final double kPassingRPS = 50.0; // Lower speed for passing

        // Hood Angles
        public static final double kShootingHoodAngle = 45.0; // degrees
        public static final double kPassingHoodAngle = 30.0; // degrees
        
        // Hood angle limits
        public static final double kMinHoodAngle = 0.0;
        public static final double kMaxHoodAngle = 60.0;
    }

    public static final class TowerConstants {
        public static final double kFeedVoltage = 12.0; // example value
        public static final double kFeedSpeed = 1.0; // example value
    }
}
