package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class TowerSubsystem extends SubsystemBase {
    private boolean isFeeding = false;

    public TowerSubsystem() {
        // Initialize motors, sensors, etc. here
    }

    private void setTowerSpeed(double speed) {
        // Code to set the speed of the tower motor
    }

    public void feedToShooter() {
        // Code to start feeding a ball up into the shooter
        setTowerSpeed(TowerConstants.kFeedSpeed);
        isFeeding = true;
    }

    public void stopFeeding() {
        // Stop the tower motor
        setTowerSpeed(0);
        isFeeding = false;
    }

    public boolean isFeeding() {
        // Return true if the tower is currently feeding a ball
        return isFeeding;
    }
}
