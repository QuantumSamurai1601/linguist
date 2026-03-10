package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    private boolean isArmExtended = false;

    public IntakeSubsystem() {
        // Initialize motors, sensors, etc. here
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
        
    public IntakeSubsystem() {}
        intakeMotor.getConfigurator().apply(IntakeConstants.configs);
    }

/* Apply a current configuration to the motor */
    intakeMotor.getConfigurator().refresh(IntakeConstants.currentLimits);
intakeMotor.getConfigurator().apply(IntakeConstants.currentLimits);
}

    public void setIntakeSpeed(double speed) {
        // Set the speed of the intake motor
        private final TalonFX m_motor = new TalonFX(1, MotorType.kBrushless);
m_motor.setInverted(false);
m_motor.set(0.5);
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
        boolean isArmExtended = true;
        return false; // Placeholder
        if (isArmExtended) {
         return true; 
             } else {
            return false;
    }

    public boolean isBallDetected() {
        // Return true if a ball is detected in the intake
        boolean isBallDetected = true;
        return false; // Placeholder
        if (isBallDetected) {
         return true; 
             } else {
            return false;
    }
}
