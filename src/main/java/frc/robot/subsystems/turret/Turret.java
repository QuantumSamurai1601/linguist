// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//-90 to -103 if -180 to 180

// we should improve our treemap for shooting
// esentially our tree map just gets a distance then uses a map to transfer that to a velocity but if its inbetween points, it will use some linear interpolation!!!

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.turret.TurretConstants;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

@Logged
public class Turret extends SubsystemBase {
  public boolean enableTracking = true;

  private final TalonFX turret = new TalonFX(TurretConstants.TurretMotorID);
  private final TalonFX hood = new TalonFX(TurretConstants.HoodMotorID);
  private final TalonFX shooterLeader = new TalonFX(TurretConstants.ShootLeadMotorID); // Left
  private final TalonFX shooterFollower = new TalonFX(TurretConstants.ShootFollowMotorID); // Right

  private final DoublePublisher ShooterLeaderRPSPublish;
  private final DoublePublisher ShooterFollowerRPSPublish;
  private final DoublePublisher HoodAnglePublish;
  private final DoublePublisher TurretAnglePublish;

  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final PositionVoltage hoodRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final Follower shooterFollowerRequest = new Follower(29, MotorAlignmentValue.Opposed);

  private final CommandSwerveDrivetrain drivetrain;

  /** Creates a new Turret. */
  public Turret(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");
    ShooterLeaderRPSPublish = table.getDoubleTopic("Shooter Leader RPS").publish();
    ShooterFollowerRPSPublish = table.getDoubleTopic("Shooter Follower RPS").publish();
    TurretAnglePublish = table.getDoubleTopic("Turret Angle").publish();
    HoodAnglePublish = table.getDoubleTopic("Hood Angle").publish();
  

    turret.getConfigurator().apply(TurretConstants.turretConfig);
    hood.getConfigurator().apply(TurretConstants.hoodConfig);
    shooterLeader.getConfigurator().apply(TurretConstants.shooterLeaderConfig);
    shooterFollower.setControl(shooterFollowerRequest);

    turret.setPosition(0);
  }

  public double convertToLegalTurretSetpointDeg(double targetAngleDeg) { // it converts -30 to 330
    targetAngleDeg =
        MathUtil.inputModulus(
            targetAngleDeg + TurretConstants.TURRET_ZERO_OFFSET_DEG, 0.0, 360.0);

    double finalOffsetAngleRot = Units.degreesToRotations(targetAngleDeg);
    var softLimits = TurretConstants.turretConfig.SoftwareLimitSwitch;

    if (softLimits.ForwardSoftLimitEnable && softLimits.ReverseSoftLimitEnable) {
      double turretForwardLimitRot = softLimits.ForwardSoftLimitThreshold;
      double turretReverseLimitRot = softLimits.ReverseSoftLimitThreshold;

      if (finalOffsetAngleRot > turretForwardLimitRot || finalOffsetAngleRot < turretReverseLimitRot) {
        var distFromUpper = Math.abs(finalOffsetAngleRot - turretForwardLimitRot);
        var distFromLower = Math.abs(finalOffsetAngleRot - turretReverseLimitRot);

        if (distFromLower <= distFromUpper) {
          finalOffsetAngleRot = turretReverseLimitRot + Units.degreesToRotations(0.05);
        } else {
          finalOffsetAngleRot = turretForwardLimitRot - Units.degreesToRotations(0.05);
        }
      }
    }

    return finalOffsetAngleRot;
  }

  public double getFieldTargetDistance(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot =
        robotPose.getTranslation()
            .plus(TurretConstants.ROBOT_TO_TURRET_METERS.rotateBy(robotPose.getRotation()));

    Translation2d turretToTarget = target.minus(turretPivot);
    return turretToTarget.getNorm();
  }

  public Rotation2d getFieldTargetAngle(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot =
      robotPose
        .getTranslation()
        .plus(TurretConstants.ROBOT_TO_TURRET_METERS.rotateBy(robotPose.getRotation()));
    Translation2d turretToTarget = target.minus(turretPivot);

    if (turretToTarget.getNorm() < 1.0e-6) {
      return Rotation2d.kZero;
    }

    return turretToTarget.getAngle().minus(robotPose.getRotation());
  }

  public void setTurretPosDeg(double deg) {
    turret.setControl(turretRequest.withPosition(convertToLegalTurretSetpointDeg(deg)));
  }

  public void setHoodPosRot(double rot) {
    double robotX = drivetrain.getState().Pose.getX();
    double blueHoodProtMin = Units.inchesToMeters(158.5);
    double blueHoodProtMax = Units.inchesToMeters(206.5);
    double redHoodProtMin = Units.inchesToMeters(445.5);
    double redHoodProtMax = Units.inchesToMeters(493);

    if ((robotX > blueHoodProtMin && robotX < blueHoodProtMax) || robotX > redHoodProtMin && robotX < redHoodProtMax) {
      hood.setControl(hoodRequest.withPosition(0));
      return;
    }

    hood.setControl(hoodRequest.withPosition(rot));
  }

  public void hoodInchUp() {
    var current = hood.getPosition().getValueAsDouble();
    setHoodPosRot(current + 0.01);
  }

  public void hoodInchDown() {
    var current = hood.getPosition().getValueAsDouble();
    setHoodPosRot(current - 0.01);
  }

  public void setShooterVel(double vel) {
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(vel));
  }

  public void trackHub() {
    Translation2d hub = TurretConstants.getHubCenterMeters().toTranslation2d();
    setTurretPosDeg(getFieldTargetAngle(hub).getDegrees());

    double distanceMeters = getFieldTargetDistance(hub);

    double shooterRps = TurretConstants.shooterTreeMap.get(distanceMeters);
    double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);

    setHoodPosRot(hoodRotations);
    setShooterVel(shooterRps);
  }

  public void trackFerry() {
    Translation2d ferry = TurretConstants.getClosestFerryPoint(drivetrain.getState().Pose.getTranslation());
    setTurretPosDeg(getFieldTargetAngle(ferry).getDegrees());

    double distanceMeters = getFieldTargetDistance(ferry);
    // double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);
    double shooterRps = TurretConstants.shooterTreeMap.get(distanceMeters);
    setHoodPosRot(0.079);
    setShooterVel(shooterRps);
  }
  
  public void setTurretTargetingMode() { //if we are closer to hub, aim at hub or else aim at ferry
    Pose2d robotPose = drivetrain.getState().Pose;
    double targetingSwitchX = AllianceFlipUtil.applyX(Units.inchesToMeters(175.0));

    if (AllianceFlipUtil.shouldFlip()) { //depends on what team we are on, it will flip the values
        if (robotPose.getX() > targetingSwitchX) {
            trackHub();
        } else {
            trackFerry();
        }
    } else {
        if (robotPose.getX() < targetingSwitchX) {
            trackHub();
        } else {
            trackFerry();
        }
    }
  }

  public void toggleTracking() { // when its true it goes of! when its off it goes true!
    if (enableTracking == true) {
      enableTracking = false;
      setTurretPosDeg(0);
      setShooterVel(30);
      setHoodPosRot(0);
    } else if (enableTracking == false) {
      enableTracking = true;
    }
  }

  public double getShooterFollowerRPS() {
    return shooterFollower.getVelocity().getValueAsDouble();
  }

  public double getShooterLeaderRPS() {
    return shooterLeader.getVelocity().getValueAsDouble();
  }

  public double getTurretAngle() {
    return Units.rotationsToDegrees(turret.getPosition().getValueAsDouble());
  }

  public double getHoodAngle() {
    return Units.rotationsToDegrees(hood.getPosition().getValueAsDouble());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ShooterLeaderRPSPublish.set(getShooterLeaderRPS());
    ShooterFollowerRPSPublish.set(getShooterFollowerRPS());
    TurretAnglePublish.set(getTurretAngle());
    HoodAnglePublish.set(getHoodAngle());

    if (enableTracking) {
      setTurretTargetingMode();
    }
  }
}
