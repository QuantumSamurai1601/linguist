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
        public static final double kShootVoltage = 12.0; // example value
        public static final double kShootingRPS = 100.0; // example value
        public static final double kPassingRPS = 0.0; // example value

        public static final double kShootingHoodAngle = 45.0; // degrees, example value
        public static final double kPassingHoodAngle = 30.0; // degrees, example value
    }

    public static final class TowerConstants {
        public static final double kFeedVoltage = 12.0; // example value
        public static final double kFeedSpeed = 1.0; // example value
    }
}
