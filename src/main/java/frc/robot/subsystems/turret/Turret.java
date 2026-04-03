// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//-90 to -103 if -180 to 180
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Turret extends SubsystemBase {
  public boolean enableTracking = true;

  private final TalonFX turret = new TalonFX(22);
  private final TalonFX hood = new TalonFX(21);
  private final TalonFX shooterLeader = new TalonFX(29); // Left
  private final TalonFX shooterFollower = new TalonFX(28); // Right

  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final PositionVoltage hoodRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final Follower shooterFollowerRequest = new Follower(29, MotorAlignmentValue.Opposed);

  private final CommandSwerveDrivetrain drivetrain;
  private final Debouncer shotReadyDebouncer =
      new Debouncer(TurretConstants.SHOT_READY_DEBOUNCE_SECONDS);

  private double turretTargetRot = 0.0;
  private double hoodTargetRot = 0.0;
  private double shooterTargetRps = 0.0;
  private boolean hoodTargetActive = false;

  /** Creates a new Turret. */
  public Turret(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    turret.getConfigurator().apply(TurretConstants.turretConfig);
    hood.getConfigurator().apply(TurretConstants.hoodConfig);
    shooterLeader.getConfigurator().apply(TurretConstants.shooterLeaderConfig);
    shooterFollower.setControl(shooterFollowerRequest);

    turret.setPosition(0);
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

  private Translation2d getTurretPivot(Pose2d robotPose) {
    return robotPose.getTranslation()
        .plus(TurretConstants.ROBOT_TO_TURRET_METERS.rotateBy(robotPose.getRotation()));
  }

  private Translation2d getTurretPivotFieldVelocity(Pose2d robotPose) {
    ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    Translation2d robotCenterFieldVelocity =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

    Translation2d turretOffset = TurretConstants.ROBOT_TO_TURRET_METERS;
    // Rotating the robot gives the turret pivot extra sideways velocity because it is offset
    // from the center of rotation.
    Translation2d pivotSpinRobotRelativeVelocity =
        new Translation2d(
            -robotRelativeSpeeds.omegaRadiansPerSecond * turretOffset.getY(),
            robotRelativeSpeeds.omegaRadiansPerSecond * turretOffset.getX());

    Translation2d pivotSpinFieldVelocity =
        pivotSpinRobotRelativeVelocity.rotateBy(robotPose.getRotation());

    return robotCenterFieldVelocity.plus(pivotSpinFieldVelocity);
  }

  private Translation2d getCompensatedFieldTarget(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot = getTurretPivot(robotPose);
    Translation2d turretPivotVelocity = getTurretPivotFieldVelocity(robotPose);

    // Start with the real field target and the turret pivot's current position.
    // This gives us the current shot distance before applying any motion compensation.
    double initialDistanceMeters = target.minus(turretPivot).getNorm();

    // Estimate how long the fuel will be in the air at this distance.
    double flightTimeSeconds = TurretConstants.flightTimeTreeMap.get(initialDistanceMeters);

    // While the fuel is flying, the turret pivot keeps moving with the robot.
    // Aim at a virtual point offset opposite that motion so the moving shot lands on the real target.
    Translation2d compensatedTarget = target.minus(turretPivotVelocity.times(flightTimeSeconds));

    // Recompute once using the led target because the shot distance changes slightly after compensation.
    double refinedDistanceMeters = compensatedTarget.minus(turretPivot).getNorm();
    double refinedFlightTimeSeconds = TurretConstants.flightTimeTreeMap.get(refinedDistanceMeters);

    // Use the refined flight time for the final compensated target.
    return target.minus(turretPivotVelocity.times(refinedFlightTimeSeconds));
  }

  public double getFieldTargetDistance(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot = getTurretPivot(robotPose);

    Translation2d turretToTarget = target.minus(turretPivot);
    return turretToTarget.getNorm();
  }

  public Rotation2d getFieldTargetAngle(Translation2d target) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d turretPivot = getTurretPivot(robotPose);
    Translation2d turretToTarget = target.minus(turretPivot);

    if (turretToTarget.getNorm() < 1.0e-6) {
      return Rotation2d.kZero;
    }

    return turretToTarget.getAngle().minus(robotPose.getRotation());
  }

  public void setTurretPosDeg(double deg) {
    turretTargetRot = convertToLegalTurretSetpointDeg(deg);
    turret.setControl(turretRequest.withPosition(turretTargetRot));
  }

  public void setHoodPosRot(double rot) {
    double robotX = drivetrain.getState().Pose.getX();
    double blueHoodProtMin = Units.inchesToMeters(158.5);
    double blueHoodProtMax = Units.inchesToMeters(206.5);
    double redHoodProtMin = Units.inchesToMeters(445.5);
    double redHoodProtMax = Units.inchesToMeters(493);

    if ((robotX > blueHoodProtMin && robotX < blueHoodProtMax) || robotX > redHoodProtMin && robotX < redHoodProtMax) {
      hoodTargetRot = 0;
      hoodTargetActive = true;
      hood.setControl(hoodRequest.withPosition(hoodTargetRot));
      return;
    }

    hoodTargetRot = rot;
    hoodTargetActive = true;
    hood.setControl(hoodRequest.withPosition(hoodTargetRot));
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
    shooterTargetRps = vel;
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(shooterTargetRps));
  }

  public boolean isReadyToShoot() {
    boolean turretReady =
        Math.abs(turret.getPosition().getValueAsDouble() - turretTargetRot)
            <= Units.degreesToRotations(TurretConstants.TURRET_READY_TOLERANCE_DEG);
    boolean hoodReady =
        !hoodTargetActive
            || Math.abs(hood.getPosition().getValueAsDouble() - hoodTargetRot)
                <= TurretConstants.HOOD_READY_TOLERANCE_ROT;
    boolean shooterReady =
        Math.abs(shooterLeader.getVelocity().getValueAsDouble() - shooterTargetRps)
            <= TurretConstants.SHOOTER_READY_TOLERANCE_RPS;
    boolean robotStable =
        Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond)
            <= TurretConstants.MAX_SHOT_READY_OMEGA_RAD_PER_SEC;

    return enableTracking
        && shotReadyDebouncer.calculate(turretReady && hoodReady && shooterReady && robotStable);
  }

  public void trackHub() {
    Translation2d hub = TurretConstants.getHubCenterMeters().toTranslation2d();
    Translation2d compensatedHub = getCompensatedFieldTarget(hub);
    setTurretPosDeg(getFieldTargetAngle(compensatedHub).getDegrees());

    double distanceMeters = getFieldTargetDistance(hub);
    double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);
    double shooterRps = TurretConstants.shooterTreeMap.get(distanceMeters);
    setHoodPosRot(hoodRotations);
    setShooterVel(shooterRps);
  }

  public void trackFerry() {
    Translation2d ferry = TurretConstants.getClosestFerryPoint(drivetrain.getState().Pose.getTranslation());
    Translation2d compensatedFerry = getCompensatedFieldTarget(ferry);
    setTurretPosDeg(getFieldTargetAngle(compensatedFerry).getDegrees());

    double distanceMeters = getFieldTargetDistance(ferry);
    // double hoodRotations = TurretConstants.hoodTreeMap.get(distanceMeters);
    double shooterRps = TurretConstants.shooterTreeMap.get(distanceMeters);
    setHoodPosRot(0.079);
    setShooterVel(shooterRps);
  }
  
  public void setTurretTargetingMode() {
    Pose2d robotPose = drivetrain.getState().Pose;
    double targetingSwitchX = AllianceFlipUtil.applyX(Units.inchesToMeters(175.0));

    if (AllianceFlipUtil.shouldFlip()) {
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

  public void toggleTracking() {
    if (enableTracking == true) {
      enableTracking = false;
      setTurretPosDeg(0);
      setShooterVel(30);
      setHoodPosRot(0);
    } else if (enableTracking == false) {
      enableTracking = true;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (enableTracking) {
      setTurretTargetingMode();
    }
  }
}
