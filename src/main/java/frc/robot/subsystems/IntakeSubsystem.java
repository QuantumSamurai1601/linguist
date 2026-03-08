package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    private boolean isArmExtended = false;

    public IntakeSubsystem() {
        // Initialize motors, sensors, etc. here
    }


    public void setIntakeSpeed(double speed) {
        // Set the speed of the intake motor
    }

    public void extendArm() {
        // Code to extend the intake arm
        setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        isArmExtended = true;
    }

    public void retractArm() {
        // Code to retract the intake arm
        setIntakeSpeed(-IntakeConstants.kIntakeSpeed);
        isArmExtended = false;
    }

    public boolean isArmExtended(){
        return isArmExtended;
    }

    public boolean isBallDetected() {
        // Return true if a ball is detected in the intake
        return false; // Placeholder
    }
}
