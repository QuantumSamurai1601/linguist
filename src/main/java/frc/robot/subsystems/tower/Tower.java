// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;



// | MEASUREMENTS | //
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;

// | configs + signals + hardware |//
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// |controls| //
import com.ctre.phoenix6.controls.NeutralOut;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.tower.TowerConstants;

public class Tower extends SubsystemBase {
  private final TalonFX tower = new TalonFX(TowerConstants.TowerMotorId);

  private final VelocityVoltage towerRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  private final DoublePublisher speedPublish;
  private final BooleanPublisher atShootPublish;
  private final BooleanPublisher atUnstuckPublish;

  private static final double RPS_tolerance = 2.0;

  /** Creates a new Tower. */
  public Tower() {
    tower.getConfigurator().apply(TowerConstants.towerConfig);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Tower");
    speedPublish    = table.getDoubleTopic("speed_rps").publish();
    atShootPublish = table.getBooleanTopic("atShoot").publish();
    atUnstuckPublish = table.getBooleanTopic("atUnstuck").publish();
  }

  public void setTowerVelocity(double vel) {
    tower.setControl(towerRequest.withVelocity(vel));
  }

  public void stopTower() {
    tower.setControl(neutral);
  }

  public Command runTowerShoot() {
    return this.runOnce(() -> this.setTowerVelocity(TowerConstants.TOWER_SHOOT_VELOCITY));
  }

  public double getRPS() {
    return tower.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(double tg_speed) {
    return Math.abs(getRPS() - tg_speed) < RPS_tolerance;
  }

  public boolean atShootSpeed() {
    return atTargetSpeed(TowerConstants.TOWER_SHOOT_VELOCITY);
  }

  public boolean atUnstuckSpeed() {
    return atTargetSpeed(TowerConstants.TOWER_UNSTUCK_VELOCITY);
  }

  public Command runTowerUnstuck() {
    return this.runOnce(() -> this.setTowerVelocity(TowerConstants.TOWER_UNSTUCK_VELOCITY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speedPublish.set(getRPS());
    atShootPublish.set(atShootSpeed());
    atUnstuckPublish.set(atUnstuckSpeed());
  }
}
