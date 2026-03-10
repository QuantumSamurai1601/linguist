package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private boolean isHoodExtended = false;

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
