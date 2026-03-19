// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//-90 to -103 if -180 to 180
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Turret extends SubsystemBase {
  private final BooleanEntry turretTrackingEntry;

  public enum State {
    UNHOMED,
    HOMING_TO_STOP,
    HOMED,
    TRACKING_HUB,
    TRACKING_FERRY
  }

  private final TalonFX turret = new TalonFX(22);
  private final TalonFX hood = new TalonFX(21);
  private final TalonFX shooterLeader = new TalonFX(29); // Left
  private final TalonFX shooterFollower = new TalonFX(28); // Right

  private final PositionVoltage turretRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final Follower shooterFollowerRequest = new Follower(29, MotorAlignmentValue.Opposed);
  private final NeutralOut neutral = new NeutralOut();

  private final Debouncer debouncer = new Debouncer(0.2);

  private final CommandSwerveDrivetrain drivetrain;
  private State state = State.UNHOMED;

  /** Creates a new Turret. */
  public Turret(CommandSwerveDrivetrain drivetrain) {
    var table = NetworkTableInstance.getDefault().getTable("turret");
    this.drivetrain = drivetrain;

    turretTrackingEntry = table.getBooleanTopic("enableTracking").getEntry(true);

    turret.getConfigurator().apply(TurretConstants.turretConfig);
    hood.getConfigurator().apply(TurretConstants.hoodConfig);
    shooterLeader.getConfigurator().apply(TurretConstants.shooterLeaderConfig);
    shooterFollower.setControl(shooterFollowerRequest);
  }

  public double convertToLegalTurretSetpointDeg(double targetAngleDeg) {
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

  public Command homeTurret() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        state = State.HOMING_TO_STOP;
        turret.getConfigurator().apply(TurretConstants.homingConfig);
        turret.setControl(new DutyCycleOut(TurretConstants.TURRET_HOMING_DUTY_CYCLE_OUT));
      }),
      new WaitUntilCommand(() ->
        debouncer.calculate(turret.getStatorCurrent().getValueAsDouble() > TurretConstants.TURRET_HOMING_STATOR_CURRENT_THRES && Math.abs(turret.getVelocity().getValueAsDouble()) < TurretConstants.TURRET_HOMING_MAX_VELOCITY_THRES)
      ).withTimeout(5),
      new InstantCommand(() -> {
        turret.setControl(neutral);
        turret.setNeutralMode(NeutralModeValue.Coast);
      }),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
        turret.setPosition(0);
        turret.setNeutralMode(NeutralModeValue.Brake);
        turret.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
        turret.getConfigurator().apply(TurretConstants.turretConfig);
        state = State.HOMED;
      })
    );
  }

  public boolean isHomed() {
    return state != State.UNHOMED && state != State.HOMING_TO_STOP;
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

  public void trackHub() {
    if (!isHomed()) {
      return;
    }
    state = State.TRACKING_HUB;
    Translation2d hub = TurretConstants.getHubCenterMeters().toTranslation2d();
    setTurretPosDeg(getFieldTargetAngle(hub).getDegrees());

    double distanceMeters = getFieldTargetDistance(hub);
    double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);
    setHoodPosRot(hoodRotations);
  }

  public void trackFerry() {
    if (!isHomed()) {
      return;
    }
    state = State.TRACKING_FERRY;
    Translation2d ferry = TurretConstants.getClosestFerryPoint(drivetrain.getState().Pose.getTranslation());
    setTurretPosDeg(getFieldTargetAngle(ferry).getDegrees());

    double distanceMeters = getFieldTargetDistance(ferry);
    double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);
    setHoodPosRot(hoodRotations);
  }

  public void setTurretPosDeg(double deg) {
    turret.setControl(turretRequest.withPosition(convertToLegalTurretSetpointDeg(deg)));
  }

  public Command homeHood() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        hood.getConfigurator().apply(TurretConstants.homingConfig);
        hood.setControl(new DutyCycleOut(TurretConstants.TURRET_HOMING_DUTY_CYCLE_OUT));
      }),
      new WaitUntilCommand(() ->
        debouncer.calculate(hood.getStatorCurrent().getValueAsDouble() > TurretConstants.TURRET_HOMING_STATOR_CURRENT_THRES && Math.abs(hood.getVelocity().getValueAsDouble()) < TurretConstants.TURRET_HOMING_MAX_VELOCITY_THRES)
      ).withTimeout(5),
      new InstantCommand(() -> {
        hood.setControl(neutral);
        hood.setNeutralMode(NeutralModeValue.Coast);
      }),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
        hood.setPosition(0);
        hood.setNeutralMode(NeutralModeValue.Brake);
        hood.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
        hood.getConfigurator().apply(TurretConstants.hoodConfig);
      })
    );
  }

  public double getFieldTargetDistance(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot =
        robotPose.getTranslation()
            .plus(TurretConstants.ROBOT_TO_TURRET_METERS.rotateBy(robotPose.getRotation()));

    Translation2d turretToTarget = target.minus(turretPivot);
    return turretToTarget.getNorm();
  }

  public void setHoodPosRot(double rot) {
    hood.setControl(hoodRequest.withPosition(rot));
  }

  public void setShooterVel(double vel) {
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(vel));
  }

  public void setTurretTargetingMode() {
    if (AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()) < AllianceFlipUtil.applyX(Units.inchesToMeters(175.0))) {
      this.trackHub();
    } else {
      this.trackFerry();
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean enableTracking = turretTrackingEntry.get();
    
    if (state != State.UNHOMED && state != State.HOMING_TO_STOP && enableTracking) {
      setTurretTargetingMode();
    }
  }
}
