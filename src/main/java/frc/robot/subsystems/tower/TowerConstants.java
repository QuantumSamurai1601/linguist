package frc.robot.subsystems.tower;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TowerConstants {
  public static final TalonFXConfiguration towerConfig = new TalonFXConfiguration();
  static {
    towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    towerConfig.CurrentLimits.SupplyCurrentLimit = 20;
    towerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    towerConfig.Slot0.kS = 0;
    towerConfig.Slot0.kV = 0;
    towerConfig.Slot0.kP = 0;
    towerConfig.Slot0.kD = 0;

    towerConfig.Feedback.SensorToMechanismRatio = 3;
  }

  public static final double TOWER_SHOOT_VELOCITY = 0;
  public static final double TOWER_UNSTUCK_VELOCITY = 0;
}
