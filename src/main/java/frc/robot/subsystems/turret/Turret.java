// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final TalonFX turret = new TalonFX(0);
  private final TalonFX hood = new TalonFX(0);
  private final TalonFX shooterLeader = new TalonFX(0); // Left
  private final TalonFX shooterFollower = new TalonFX(0); // Right

  private final PositionVoltage turretRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final PositionVoltage hoodRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final Follower shooterFollowerRequest = new Follower(0, MotorAlignmentValue.Opposed);

  /** Creates a new Turret. */
  public Turret() {
    turret.getConfigurator().apply(TurretConstants.turretConfig);
    hood.getConfigurator().apply(TurretConstants.hoodConfig);
    shooterLeader.getConfigurator().apply(TurretConstants.shooterLeaderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
