package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.AllianceFlipUtil;
import frc.robot.subsystems.vision.VisionConstants;

public class TurretConstants {
  public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  static {
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Change after tuning

    turretConfig.CurrentLimits.SupplyCurrentLimit = 20;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.923;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // Change after tuning
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // Change after turning

    turretConfig.Slot0.kS = 0.28;
    turretConfig.Slot0.kP = 50;
    turretConfig.Slot0.kD = 0;
    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    turretConfig.MotionMagic.MotionMagicAcceleration = 4.5;

    turretConfig.Feedback.SensorToMechanismRatio = 35;
  }

  public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  static {
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Change after tuning

    hoodConfig.CurrentLimits.SupplyCurrentLimit = 10;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.085;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.Slot0.kS = 0.385;
    hoodConfig.Slot0.kP = 1000;
    hoodConfig.Slot0.kD = 0;

    hoodConfig.Feedback.SensorToMechanismRatio = 214.3;
  }

  public static final TalonFXConfiguration shooterLeaderConfig = new TalonFXConfiguration();
  static {
    shooterLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterLeaderConfig.Slot0.kS = 0.24;
    shooterLeaderConfig.Slot0.kV = 0.475;
    shooterLeaderConfig.Slot0.kP = 0.85;
    shooterLeaderConfig.Slot0.kD = 0;

    shooterLeaderConfig.Feedback.SensorToMechanismRatio = 1.64;
  }

  public static final InterpolatingDoubleTreeMap shooterTreeMap = new InterpolatingDoubleTreeMap();
  static {
    shooterTreeMap.put(1.77, 28.0);
    shooterTreeMap.put(2.82, 34.0);
    shooterTreeMap.put(3.79, 39.0);
    shooterTreeMap.put(4.75, 44.0);
    shooterTreeMap.put(5.75, 60.0);
  }

  public static final InterpolatingDoubleTreeMap hoodTreeMap = new InterpolatingDoubleTreeMap();
  static {
    hoodTreeMap.put(1.77, 0.0);
    hoodTreeMap.put(2.82, 0.0);
    hoodTreeMap.put(3.79, 0.0);
  }

  public static final TalonFXConfiguration homingConfig = new TalonFXConfiguration();
  static {
    homingConfig.CurrentLimits.StatorCurrentLimit = 30;
    homingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    homingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    homingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }

  public static final double TURRET_HOMING_DUTY_CYCLE_OUT = 0.1;
  public static final double TURRET_HOMING_STATOR_CURRENT_THRES = 10;
  public static final double TURRET_HOMING_MAX_VELOCITY_THRES = 1;

  public static final double TURRET_ZERO_OFFSET_DEG = 90.0;
  public static final Translation2d ROBOT_TO_TURRET_METERS = new Translation2d(Units.inchesToMeters(5.21), 0);

  public static Translation3d getHubCenterMeters() {
    return AllianceFlipUtil.apply(new Translation3d(
      VisionConstants.aprilTagLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47.0) / 2.0,
      VisionConstants.aprilTagLayout.getFieldWidth() / 2.0,
      Units.inchesToMeters(72)));
  }

  public static Translation2d getClosestFerryPoint(Translation2d current) {
    // Also technically upper red point
    Translation2d lowerBluePoint = AllianceFlipUtil.apply(new Translation2d(
      Units.inchesToMeters(45),
      VisionConstants.aprilTagLayout.getFieldWidth() / 4.0
    ));
    // Also technically lower red point
    Translation2d higherBluePoint = AllianceFlipUtil.apply(new Translation2d(
      Units.inchesToMeters(45),
      (VisionConstants.aprilTagLayout.getFieldWidth() / 4.0) * 3.0
    ));

    Translation2d closerPoint =
    current.getDistance(lowerBluePoint) < current.getDistance(higherBluePoint) ? lowerBluePoint : higherBluePoint;

    return closerPoint;
  }

  public static final double TURRET_SHOOTING_RPS = 50;

  public static final int TurretMotorID = 22;
  public static final int HoodMotorID = 21;
  public static final int ShootLeadMotorID = 29;
  public static final int ShootFollowMotorID = 28;
}
