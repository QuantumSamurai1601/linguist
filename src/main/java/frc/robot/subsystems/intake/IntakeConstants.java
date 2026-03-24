package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  public static final TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
  static {
    intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeRollerConfig.Feedback.SensorToMechanismRatio = 2;
  }

  public static final TalonFXConfiguration intakeExtendConfig = new TalonFXConfiguration();
  static {
    intakeExtendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeExtendConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeExtendConfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeExtendConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeExtendConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    intakeExtendConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    intakeExtendConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // Change after tuning
    intakeExtendConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // Change after turning

    intakeExtendConfig.Slot0.kS = 0.43;
    intakeExtendConfig.Slot0.kP = 300;
    intakeExtendConfig.Slot0.kD = 0;

    intakeExtendConfig.Feedback.SensorToMechanismRatio = 66;
  }

  public static final TalonFXConfiguration homingConfig = new TalonFXConfiguration();
  static {
    homingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    homingConfig.CurrentLimits.StatorCurrentLimit = 50;
    homingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    homingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    homingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }

  public static final double INTAKING_VOLTS = 7.6;
  public static final double OUTAKING_VOLTS = -7.6;
  public static final double INTAKE_EXTEND_ASSIST_TIME_SEC = 0.5;
  public static final double INTAKE_STOW_POS = 0.32;
  public static final double INTAKE_EXTEND_POS = 0.73;

  public static final double INTAKE_HOMING_DUTY_CYCLE_OUT = 0.25;
  public static final double INTAKE_HOMING_STATOR_CURRENT_THRES = 30;
  public static final double INTAKE_HOMING_MAX_VELOCITY_THRES = 0.72;
}
