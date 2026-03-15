// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class Tower extends SubsystemBase {
  private final TalonFX tower = new TalonFX(0);

  private final VelocityVoltage towerRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  /** Creates a new Tower. */
  public Tower() {
    tower.getConfigurator().apply(TowerConstants.towerConfig);
  }

    public void setTowerVelocity(double vel) {
    tower.setControl(towerRequest.withVelocity(vel));
  }

  public void stopTower() {
    tower.setControl(neutral);
  }

  public Command runTowerShoot() {
    return this.runOnce(() -> this.setTowerVelocity(HopperConstants.HOPPER_SHOOT_VELOCITY));
  }

  public Command runTowerUnstuck() {
    return this.runOnce(() -> this.setTowerVelocity(HopperConstants.HOPPER_UNSTUCK_VELOCITY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
