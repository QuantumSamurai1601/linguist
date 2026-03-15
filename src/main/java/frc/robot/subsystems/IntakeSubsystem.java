package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertedValue;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static final TalonFX armMotor = new TalonFX(Constants.Intake.Arm.deviceID);
    private static final TalonFXSimState armMotorSim = armMotor.getSimState();

    private static final TalonFX rollerMotor = new TalonFX(Constants.Intake.Roller.deviceID);
    private static final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();

    private static IntakeSubsystem sInstance = null;

    public static IntakeSubsystem getInstance() {
        if (sInstance == null) {
            sInstance = new IntakeSubsystem();
        }
        return sInstance;
    }

    private boolean isArmExtended = false;

    public IntakeSubsystem() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(rollerConfig);
        armMotor.setPosition(Constants.Intake.Arm.startingAngle);
        retractArm();
    }

    public void setIntakeSpeed(double speed) {
        rollerMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void extendArm() {
        setIntakeSpeed(Constants.Intake.Roller.kIntakeSpeed);
        isArmExtended = true;
    }

    public void retractArm() {
        setIntakeSpeed(0.0);
        armMotor.setPosition(Constants.Intake.Arm.stowAngle);
        isArmExtended = false;
    }

    public boolean isArmExtended() {
        return isArmExtended;
    }

   public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
}

public boolean isBallDetected() {
    return false;
}
    private double m_lastRollerSpeed = 0.0;

public void setIntakeSpeed(double speed) {
    rollerMotor.set(TalonFXControlMode.PercentOutput, speed);
    m_lastRollerSpeed = speed;
}

public boolean isIntakeRunning() {
    return Math.abs(m_lastRollerSpeed) > 0.1;
}

public boolean isIdle() {
    return !isArmExtended() && !isIntakeRunning();
}
