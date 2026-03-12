package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
private static final TalonFX armMotor = new TalonFX(Constants.Intake.Arm.deviceID);
private static final TalonFXSimState armMotorSim = armMotor.getSimState();

private static final TalonFX rollerMotor = new TalonFX(Constants.Intake.Roller.deviceID);
private static final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();
    
    private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
    }
        return sInstance;
    }

    public IntakeSubsystem() {
        // Initialize motors, sensors, etc. here
        intakeMotor.getConfigurator().apply(IntakeConstants.configs);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
        
/* Apply a current configuration to the motor */
    intakeMotor.getConfigurator().refresh(IntakeConstants.currentLimits);
intakeMotor.getConfigurator().apply(IntakeConstants.currentLimits);
         TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(rollerConfig);

    armMotor.setPosition(Constants.Intake.Arm.startingAngle);
        retract();
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
        setPosition(Constants.Intake.Arm.stowAngle);
        rollerMotor.set(0.0);
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
