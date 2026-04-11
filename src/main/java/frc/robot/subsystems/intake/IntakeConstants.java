package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  public static final TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
  static {
    intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeRollerConfig.Feedback.SensorToMechanismRatio = 2;
  }

  public static final TalonFXConfiguration intakeExtendConfig = new TalonFXConfiguration();
  static {
    intakeExtendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeExtendConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeExtendConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeExtendConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeExtendConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeExtendConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeExtendConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    intakeExtendConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.01;
    intakeExtendConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // Change after tuning
    intakeExtendConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // Change after turning

    intakeExtendConfig.Slot0.kS = 0.44;
    intakeExtendConfig.Slot0.kP = 350;
    intakeExtendConfig.Slot0.kD = 0;
    intakeExtendConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    intakeExtendConfig.MotionMagic.MotionMagicAcceleration = 0;

    intakeExtendConfig.Feedback.SensorToMechanismRatio = 66;
  }

  public static final double INTAKING_VOLTS = 5.67;
  public static final double OUTAKING_VOLTS = -5.67;
  public static final double INTAKE_EXTEND_ASSIST_TIME_SEC = 0.1;
  public static final double INTAKE_STOW_POS = 0.23;
  public static final double INTAKE_EXTEND_POS = 0.645;
  public static final double INTAKE_AGITATE_POS = 0.01;
  public static final double INTAKE_AGITATE_MOVE_TIME_SEC = 0.59;
}
