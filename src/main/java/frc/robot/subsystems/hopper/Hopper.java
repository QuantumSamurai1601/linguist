// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final TalonFX hopper = new TalonFX(34);
  
  private final VelocityVoltage hopperRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  /** Creates a new Indexer. */
  public Hopper() {
    hopper.getConfigurator().apply(HopperConstants.hopperConfig);
  }

  public void setHopperVelocity(double vel) {
    hopper.setControl(hopperRequest.withVelocity(vel));
  }

  public void stopHopper() {
    hopper.setControl(neutral);
  }

  public Command runHopperIntake() {
    return this.runOnce(() -> this.setHopperVelocity(HopperConstants.HOPPER_INTAKE_VELOCITY));
  }

  public Command runHopperShoot() {
    return this.runOnce(() -> this.setHopperVelocity(HopperConstants.HOPPER_SHOOT_VELOCITY));
  }

  public Command runHopperUnstuck() {
    return this.runOnce(() -> this.setHopperVelocity(HopperConstants.HOPPER_UNSTUCK_VELOCITY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
