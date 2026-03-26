// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Hopper extends SubsystemBase {
  private final TalonFX hopper = new TalonFX(HopperConstants.HopperMotorID);
  
  private final VelocityVoltage hopperRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  private static final double RPS_TOLERANCE = 2.0;

  private final DoublePublisher speedPublish;
  private final BooleanPublisher AtIntakePublish;
  private final BooleanPublisher AtShootPublish;
  private final BooleanPublisher AtUnstuckPublish;


  /** Creates a new Indexer. */
  public Hopper() {
    hopper.getConfigurator().apply(HopperConstants.hopperConfig);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Hopper");
      speedPublish    = table.getDoubleTopic("speed_rps").publish();
      AtIntakePublish = table.getBooleanTopic("AtIntake").publish();
      AtShootPublish = table.getBooleanTopic("atShoot").publish();
      AtUnstuckPublish = table.getBooleanTopic("atUnstuck").publish();
  }

  public void setHopperVelocity(double vel) {
    hopper.setControl(hopperRequest.withVelocity(vel));
  }

  public double getHopperVelocity(){
    return hopper.getVelocity().getValueAsDouble();
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

  public boolean atTargetSpeed(double targetSpeed){
    return Math.abs(getHopperVelocity() - targetSpeed) < RPS_TOLERANCE;
  }

  public boolean atIntakeSpeed(){
    return atTargetSpeed(HopperConstants.HOPPER_INTAKE_VELOCITY);
  }

  public boolean atShootSpeed(){
    return atTargetSpeed(HopperConstants.HOPPER_SHOOT_VELOCITY);
  }

  public boolean atUnstuckSpeed(){
    return atTargetSpeed(HopperConstants.HOPPER_UNSTUCK_VELOCITY);
  }

  @Override
  public void periodic() {
    speedPublish.set(getHopperVelocity());
    AtIntakePublish.set(atIntakeSpeed());
    AtShootPublish.set(atShootSpeed());
    AtUnstuckPublish.set(atUnstuckSpeed());
  }
}
