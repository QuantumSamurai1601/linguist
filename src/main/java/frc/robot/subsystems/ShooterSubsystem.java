package frc.robot.subsystems;

// || IMPORTS 

// phoenix6 motor imports //
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// measurements //
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// geometry + math //
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// files inside robot //
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.*;
// || END OF IMPORTS

public class ShooterSubsystem extends SubsystemBase {
    private boolean isHoodExtended = false;
        public boolean enableTracking = true;

        private final TalonFX turret = new TalonFX(Constants.ShooterConstants.TurretMotorId);
        private final TalonFX hood = new TalonFX(Constants.ShooterConstants.HoodMotorId);
        private final TalonFX shooterLeader = new TalonFX(Constants.ShooterConstants.ShooterLeftMotorId); // Left
        private final TalonFX shooterFollower = new TalonFX(Constants.ShooterConstants.ShooterRightMotorId); // Right

    public ShooterSubsystem() {
        // Initialize shooter hardware (motors, encoders, etc.)

    }

    public void setSpeed(double rps) {
        // Set the flywheel speed in revolutions per second
    }

    public boolean isAtSpeed() {
        // Return true if the flywheel is at the target speed
        return false; // Placeholder
    }

    public void stop() {
        // Stop the shooter motors
    }

    public void coast() {
        // Set the shooter motors to coast mode
        stop();
    }

    public void setHoodAngle(double angle) {
        // Set the hood angle in degrees
    }

    public void retractHood(){
        // Retract the hood to the passing position
        setHoodAngle(ShooterConstants.kPassingHoodAngle);
        isHoodExtended = false;
    }

    public void extendHood() {
        // Extend the hood to the shooting position
        setHoodAngle(ShooterConstants.kShootingHoodAngle);
        isHoodExtended = true;
    }

    public boolean isHoodExtended() {
        // Return true if the hood is extended to the shooting position
        return isHoodExtended;
    }
    
}
