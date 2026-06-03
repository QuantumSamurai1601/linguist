// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//-90 to -103 if -180 to 180
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.AllianceFlipUtil;
import frc.robot.HubShiftUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Turret extends SubsystemBase {
  public enum TrackingState {
    HUB,
    FERRY,
    NONE
  }
  private NetworkTable turretTable = NetworkTableInstance.getDefault().getTable("Turret");
  private BooleanPublisher isTrackingPub = turretTable.getBooleanTopic("Tracking Enabled").publish();
  private StringPublisher trackedTargetPub = turretTable.getStringTopic("Tracked Target").publish();
  // private StringPublisher distanceToTrackedPub = turretTable.getStringTopic("Tracked Target Dist.").publish();
  // private StringPublisher hoodPosRotPub = turretTable.getStringTopic("Hood Position").publish();
  // private StringPublisher shooterVelRpsPub = turretTable.getStringTopic("Shooter Speed").publish();
  private BooleanPublisher turretReadyPub = turretTable.getBooleanTopic("Turret Ready?").publish();
  private BooleanPublisher hubActiveOrFerryPub = turretTable.getBooleanTopic("Hub Active OR Ferry").publish();
  private BooleanPublisher canShootPub = turretTable.getBooleanTopic("Can Shoot?").publish();

  public boolean enableTracking = true;
  public boolean enableHood = false;
  public TrackingState trackingTarget = TrackingState.NONE;
  public double distanceToTrackedTarget = 0.0;
  public boolean turretReady = false;
  public boolean hubActiveOrFerrying = false;
  public boolean canShoot = false;

  private final TalonFX turret = new TalonFX(22);
  private final TalonFX hood = new TalonFX(21);
  private final TalonFX shooterLeader = new TalonFX(29); // Left
  private final TalonFX shooterFollower = new TalonFX(28); // Right

  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final PositionVoltage hoodRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final Follower shooterFollowerRequest = new Follower(29, MotorAlignmentValue.Opposed);

  private final StatusSignal<Boolean> turretForwardSoftLimit = turret.getFault_ForwardSoftLimit(false);
  private final StatusSignal<Boolean> turretReverseSoftLimit = turret.getFault_ReverseSoftLimit(false);
  private final StatusSignal<Double> turretClosedLoopError = turret.getClosedLoopError(false);

  private final Debouncer canShootDebounce = new Debouncer(0.1);
  private final CommandSwerveDrivetrain drivetrain;
  private SwerveDriveState driveState;

  private double turretTargetRot = 0.0;
  private double hoodTargetRot = 0.0;
  private double shooterTargetRps = 0.0;

  private double hoodDesiredRot = 0.0;

  /** Creates a new Turret. */
  public Turret(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    driveState = drivetrain.getState();

    turret.getConfigurator().apply(TurretConstants.turretConfig);
    hood.getConfigurator().apply(TurretConstants.hoodConfig);
    shooterLeader.getConfigurator().apply(TurretConstants.shooterLeaderConfig);
    shooterFollower.setControl(shooterFollowerRequest);

    turret.setPosition(0);
  }

  public double convertDegToLegalTurretSetpointRot(double targetAngleDeg) {
    targetAngleDeg =
        MathUtil.inputModulus(
            targetAngleDeg + TurretConstants.TURRET_ZERO_OFFSET_DEG, 0.0, 360.0);

    double finalOffsetAngleRot = Units.degreesToRotations(targetAngleDeg);
    var softLimits = TurretConstants.turretConfig.SoftwareLimitSwitch;

    if (turretForwardSoftLimit.getValue() || turretReverseSoftLimit.getValue()) {
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
    ChassisSpeeds robotRelativeSpeeds = driveState.Speeds;
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

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
    Pose2d robotPose = driveState.Pose;
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
    Pose2d robotPose = driveState.Pose;
    Translation2d turretPivot = getTurretPivot(robotPose);

    Translation2d turretToTarget = target.minus(turretPivot);
    return turretToTarget.getNorm();
  }

  public Rotation2d getFieldTargetAngle(Translation2d target) {
    Pose2d robotPose = driveState.Pose;
    Translation2d turretPivot = getTurretPivot(robotPose);
    Translation2d turretToTarget = target.minus(turretPivot);

    if (turretToTarget.getNorm() < 1.0e-6) {
      return Rotation2d.kZero;
    }

    return turretToTarget.getAngle().minus(robotPose.getRotation());
  }

  public void setTurretPosDeg(double deg) {
    turretTargetRot = convertDegToLegalTurretSetpointRot(deg);
    turret.setControl(turretRequest.withPosition(turretTargetRot));
  }

  public void setHoodPosRot() {
    Pose2d pose = driveState.Pose;
    double robotX = pose.getX();
    double robotY = pose.getY();
    double blueHoodProtMin = Units.inchesToMeters(157.11); // Blue trench X line at 182.11 +/- 25
    double blueHoodProtMax = Units.inchesToMeters(207.11);
    double redHoodProtMin = Units.inchesToMeters(444.11); // Red trench X line at 469.11 +/- 25
    double redHoodProtMax = Units.inchesToMeters(494.11);
    double lowerHoodYProt = Units.inchesToMeters(50.67);
    double upperHoodYProt = Units.inchesToMeters(267.02);

    if ((robotX > blueHoodProtMin && robotX < blueHoodProtMax && (robotY < lowerHoodYProt || robotY > upperHoodYProt))   // Blue alliance trenches and in Y
        || robotX > redHoodProtMin && robotX < redHoodProtMax && (robotY < lowerHoodYProt || robotY > upperHoodYProt)) { // Red alliance trenches and in Y
      hoodTargetRot = 0;
      hood.setControl(hoodRequest.withPosition(hoodTargetRot));
      return;
    }

    hoodTargetRot = hoodDesiredRot;
    hood.setControl(hoodRequest.withPosition(hoodTargetRot));
  }

  public void setHoodPosRot(double rot) {
    Pose2d pose = driveState.Pose;
    double robotX = pose.getX();
    double robotY = pose.getY();
    double blueHoodProtMin = Units.inchesToMeters(155.11); // Blue trench X line at 182.11 +/- 27
    double blueHoodProtMax = Units.inchesToMeters(209.11);
    double redHoodProtMin = Units.inchesToMeters(442.11); // Red trench X line at 469.11 +/- 27
    double redHoodProtMax = Units.inchesToMeters(496.11);
    double lowerHoodYProt = Units.inchesToMeters(50.67);
    double upperHoodYProt = Units.inchesToMeters(267.02);

    if ((robotX > blueHoodProtMin && robotX < blueHoodProtMax && (robotY < lowerHoodYProt || robotY > upperHoodYProt))   // Blue alliance trenches and in Y
        || robotX > redHoodProtMin && robotX < redHoodProtMax && (robotY < lowerHoodYProt || robotY > upperHoodYProt)) { // Red alliance trenches and in Y
      hoodTargetRot = 0;
      hood.setControl(hoodRequest.withPosition(hoodTargetRot));
      return;
    }

    hoodTargetRot = rot;
    hood.setControl(hoodRequest.withPosition(hoodTargetRot));
  }

  public void hoodInchUp() {
    var current = hood.getPosition().getValueAsDouble();
    setHoodPosRot(current + 0.005);
  }

  public void hoodInchDown() {
    var current = hood.getPosition().getValueAsDouble();
    setHoodPosRot(current - 0.005);
  }

  public void toggleHood(boolean turnHoodOn) {
    if (turnHoodOn && enableTracking) {
      enableHood = true;
      return;
    } else if (turnHoodOn) {
      setHoodPosRot(0.04);
      return;
    } else if (!turnHoodOn) {
      enableHood = false;
      setHoodPosRot(0);
    }
  }

  public void setShooterVel(double vel) {
    shooterTargetRps = vel;
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(shooterTargetRps));
  }

  public void shooterNudgeUp() {
    var current = shooterLeader.getClosedLoopReference().getValueAsDouble();
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(current + 2));
  }

  public void shooterNudgeDown() {
    var current = shooterLeader.getClosedLoopReference().getValueAsDouble();
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(current - 2));
  }

  public void trackHub() {
    Translation2d hub = TurretConstants.getHubCenterMeters().toTranslation2d();
    Translation2d compensatedHub = getCompensatedFieldTarget(hub);
    setTurretPosDeg(getFieldTargetAngle(compensatedHub).getDegrees());

    distanceToTrackedTarget = getFieldTargetDistance(compensatedHub);

    double shooterRps = TurretConstants.shooterTreeMap.get(distanceToTrackedTarget);  
    setShooterVel(shooterRps);

    hoodDesiredRot = TurretConstants.hoodTreeMap.get(distanceToTrackedTarget);
  }

  public void trackFerry() {
    Translation2d ferry = TurretConstants.getClosestFerryPoint(driveState.Pose.getTranslation());
    Translation2d compensatedFerry = getCompensatedFieldTarget(ferry);
    setTurretPosDeg(getFieldTargetAngle(compensatedFerry).getDegrees());

    distanceToTrackedTarget = getFieldTargetDistance(compensatedFerry);
    
    double shooterRps = TurretConstants.shooterTreeMap.get(distanceToTrackedTarget);
    setShooterVel(shooterRps);

    hoodDesiredRot = 0.079;
  }
  
  public void setTurretTargetingMode() {
    double robotPoseX = driveState.Pose.getX();
    double targetingSwitchX = AllianceFlipUtil.applyX(Units.inchesToMeters(180.0));
    boolean isRedAlliance = AllianceFlipUtil.shouldFlip();

    boolean targetHub = isRedAlliance ? (robotPoseX > targetingSwitchX) : (robotPoseX < targetingSwitchX);

    if (targetHub) {
      trackHub();
      trackingTarget = TrackingState.HUB;
    } else {
      trackFerry();
      trackingTarget = TrackingState.FERRY;
    }
  }

  public void toggleTracking() {
    if (enableTracking == true) {
      enableTracking = false;
      enableHood = false;
      setTurretPosDeg(0);
      setShooterVel(31);
      setHoodPosRot(0);
    } else if (enableTracking == false) {
      enableTracking = true;
    }
  }

  public boolean readyToShoot() {
    turretReady = Math.abs(turretClosedLoopError.getValueAsDouble()) < TurretConstants.TURRET_READY_TOLERANCE_ROT;

    hubActiveOrFerrying = (trackingTarget == TrackingState.HUB && HubShiftUtil.getShiftedShiftInfo().active())
        || trackingTarget == TrackingState.FERRY
            && !(driveState.Pose.getY() > Units.inchesToMeters(135.34) 
                && driveState.Pose.getY() < Units.inchesToMeters(182.34));

    canShoot = canShootDebounce.calculate(turretReady && hubActiveOrFerrying);
    return canShoot;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveState = drivetrain.getState();

    BaseStatusSignal.refreshAll(
      turretForwardSoftLimit,
      turretReverseSoftLimit,
      turretClosedLoopError
    );

    if (enableTracking) {setTurretTargetingMode();}
    if (enableHood) {setHoodPosRot();}

    isTrackingPub.set(enableTracking);
    trackedTargetPub.set(trackingTarget.toString());
    // distanceToTrackedPub.set(String.format("%.2f", distanceToTrackedTarget));
    // hoodPosRotPub.set(String.format("%.3f", hood.getPosition().getValueAsDouble()));
    // shooterVelRpsPub.set(String.format("%.2f", shooterLeader.getVelocity().getValueAsDouble()));

    turretReadyPub.set(turretReady);
    hubActiveOrFerryPub.set(hubActiveOrFerrying);
    canShootPub.set(canShoot);
  }
}
