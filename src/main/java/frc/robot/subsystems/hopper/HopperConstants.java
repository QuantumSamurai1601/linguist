package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConstants {
  public static final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
  static {
    hopperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    hopperConfig.CurrentLimits.SupplyCurrentLimit = 20;
    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hopperConfig.Slot0.kS = 0;
    hopperConfig.Slot0.kV = 0;
    hopperConfig.Slot0.kP = 0;
    hopperConfig.Slot0.kD = 0;

    hopperConfig.Feedback.SensorToMechanismRatio = 2;
  }

  public static final double HOPPER_INTAKE_VELOCITY = 0;
  public static final double HOPPER_SHOOT_VELOCITY = 0;
  public static final double HOPPER_UNSTUCK_VELOCITY = 0;
}
