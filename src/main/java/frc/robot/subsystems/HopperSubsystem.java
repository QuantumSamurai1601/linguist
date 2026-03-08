package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class HopperSubsystem extends SubsystemBase {

    private boolean isRunning = false;

    public HopperSubsystem() {
        // Initialize motors, sensors, etc. here
    }

    private void setHopperSpeed(double speed) {
        // Code to set the speed of the hopper motor
    }

    public void runHopper() {
       // Code to deploy the hopper mechanism
        setHopperSpeed(HopperConstants.kHopperSpeed); // Example speed value
        isRunning = true;
    }

    public void stopHopper() {
        // Stop the hopper motor
        setHopperSpeed(0);
        isRunning = false;
    }

    public boolean isRunning() {
        // Return true if the hopper is running
        return isRunning;
    }
}
