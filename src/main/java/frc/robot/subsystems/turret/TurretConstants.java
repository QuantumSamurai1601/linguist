package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TurretConstants {
  public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  static {
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Change after tuning

    turretConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // Change after tuning
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // Change after turning

    turretConfig.Slot0.kS = 0;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kD = 0;

    turretConfig.Feedback.SensorToMechanismRatio = 35;
  }

  public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  static {
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Change after tuning

    hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // Change after tuning
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // Change after turning

    hoodConfig.Slot0.kS = 0;
    hoodConfig.Slot0.kP = 0;
    hoodConfig.Slot0.kD = 0;

    hoodConfig.Feedback.SensorToMechanismRatio = 214.3;
  }

  public static final TalonFXConfiguration shooterLeaderConfig = new TalonFXConfiguration();
  static {
    shooterLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterLeaderConfig.CurrentLimits.SupplyCurrentLimit = 20;
    shooterLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterLeaderConfig.Slot0.kS = 0;
    shooterLeaderConfig.Slot0.kV = 0;
    shooterLeaderConfig.Slot0.kP = 0;
    shooterLeaderConfig.Slot0.kD = 0;

    shooterLeaderConfig.Feedback.SensorToMechanismRatio = 1.64;
  }

  public static final InterpolatingDoubleTreeMap shooterTreeMap = new InterpolatingDoubleTreeMap();
  static {
      shooterTreeMap.put(0.0, 0.175);
      shooterTreeMap.put(0.5, 0.13);
      shooterTreeMap.put(1.0, 0.115);
      shooterTreeMap.put(1.5, 0.115);
      shooterTreeMap.put(2.0, 0.115);
      shooterTreeMap.put(2.5, 0.115);
      shooterTreeMap.put(3.0, 0.115);
  }

  public static final InterpolatingDoubleTreeMap hoodTreeMap = new InterpolatingDoubleTreeMap();
  static {
      hoodTreeMap.put(0.0, 0.175);
      hoodTreeMap.put(0.5, 0.13);
      hoodTreeMap.put(1.0, 0.115);
      hoodTreeMap.put(1.5, 0.115);
      hoodTreeMap.put(2.0, 0.115);
      hoodTreeMap.put(2.5, 0.115);
      hoodTreeMap.put(3.0, 0.115);
  }

  public static final TalonFXConfiguration homingConfig = new TalonFXConfiguration();
  static {
    homingConfig.CurrentLimits.StatorCurrentLimit = 30;
    homingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    homingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    homingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }

  public static final double INTAKE_HOMING_DUTY_CYCLE_OUT = 0.1;
  public static final double INTAKE_HOMING_STATOR_CURRENT_THRES = 10;
  public static final double INTAKE_HOMING_MAX_VELOCITY_THRES = 1;
}