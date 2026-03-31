// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Intake extends SubsystemBase {
  private final TalonFX intakeRoller = new TalonFX(44);
  private final TalonFX intakeExtender = new TalonFX(45);

  private final VoltageOut intakeRollerRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage intakeExtendRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  private final Debouncer debouncer = new Debouncer(0.1);

  public boolean isIntakeWheelOn = false;
  public boolean isIntakeExtended = false;
  public boolean hasIntakeHomed = false;

  public DoublePublisher speedPubExtender;
  public BooleanPublisher atTargetPubExtender;

  public DoublePublisher speedPubRoller;
  public BooleanPublisher atTargetPubRoller;
    
  public Intake() {
    intakeRoller.getConfigurator().apply(IntakeConstants.intakeRollerConfig);
    intakeExtender.getConfigurator().apply(IntakeConstants.intakeExtendConfig);

    intakeExtender.setPosition(0);

    NetworkTable extenderTable = NetworkTableInstance.getDefault().getTable("IntakeExtender");
    speedPubExtender = extenderTable.getDoubleTopic("Speed").publish();
    atTargetPubExtender = extenderTable.getBooleanTopic("AtTargetExtender").publish();

    NetworkTable rollerTable = NetworkTableInstance.getDefault().getTable("IntakeRoller");

    speedPubRoller = rollerTable.getDoubleTopic("Speed").publish();
    atTargetPubRoller = rollerTable.getBooleanTopic("AtTargetRoller").publish();

  }

  public double getIntakeExtendPosition() {
    return intakeExtender.getPosition().getValueAsDouble();
   
  }

  public double getIntakeRollerPosition() {
    return intakeRoller.getPosition().getValueAsDouble();
  }

  public double getIntakeRollerVelocity() {
    return intakeRoller.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeedExtender() {
    return Math.abs(intakeRoller.getVelocity().getValueAsDouble() - IntakeConstants.EXTENDER_VELOCITY_TOLERANCE_RPS) < IntakeConstants.EXTENDER_VELOCITY_TOLERANCE_RPS;
  }

  public boolean atTargetSpeedRoller() {
    return Math.abs(intakeRoller.getVelocity().getValueAsDouble() - IntakeConstants.ROLLER_VELOCITY_TOLERANCE_RPS) < IntakeConstants.ROLLER_VELOCITY_TOLERANCE_RPS;
  }

  
  public void setIntakeVolts(double volts) {
    intakeRoller.setControl(intakeRollerRequest.withOutput(volts));
  }

  public void toggleIntake() {
    if (isIntakeWheelOn == false) {
      this.setIntakeVolts(IntakeConstants.INTAKING_VOLTS);
      isIntakeWheelOn = true;
    } else if (isIntakeWheelOn == true) {
      this.setIntakeNeutral();
      isIntakeWheelOn = false;
    }
  }

  public void setIntakeNeutral() {
    isIntakeWheelOn = false;
    intakeRoller.setControl(neutral);
  }

  private void setIntakePos(double pos) {
    intakeExtender.setControl(intakeExtendRequest.withPosition(pos));
  }

  public Command homeIntakeExtend() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        intakeExtender.getConfigurator().apply(IntakeConstants.homingConfig);
        intakeExtender.setControl(new DutyCycleOut(IntakeConstants.INTAKE_HOMING_DUTY_CYCLE_OUT));
      }),
      new WaitUntilCommand(() ->
        debouncer.calculate(intakeExtender.getStatorCurrent().getValueAsDouble() > IntakeConstants.INTAKE_HOMING_STATOR_CURRENT_THRES && Math.abs(intakeExtender.getVelocity().getValueAsDouble()) < IntakeConstants.INTAKE_HOMING_MAX_VELOCITY_THRES)
      ).withTimeout(2),
      new InstantCommand(() -> {
        intakeExtender.setControl(neutral);
        intakeExtender.setNeutralMode(NeutralModeValue.Brake);
      }),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
        intakeExtender.setPosition(0);
        hasIntakeHomed = true;
        intakeExtender.setNeutralMode(NeutralModeValue.Coast);
        intakeExtender.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
        intakeExtender.getConfigurator().apply(IntakeConstants.intakeExtendConfig);
        intakeExtender.setControl(intakeExtendRequest.withPosition(IntakeConstants.INTAKE_STOW_POS));
      })
    );
  }
  
  public boolean getHasIntakeHomed() {
    return this.hasIntakeHomed;
  }

  public Command extendIntake() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        this.setIntakePos(IntakeConstants.INTAKE_EXTEND_POS);
        this.isIntakeExtended = true;
        if (intakeExtender.getPosition().getValueAsDouble() < 0.1) {
          this.setIntakeVolts(IntakeConstants.OUTAKING_VOLTS);
        }
      }),

      new WaitCommand(IntakeConstants.INTAKE_EXTEND_ASSIST_TIME_SEC),

      new InstantCommand(() -> this.setIntakeNeutral()),

      new InstantCommand(() -> {
        this.setIntakeVolts(IntakeConstants.INTAKING_VOLTS);
        this.isIntakeWheelOn = true;
      })
    );
  }



  public Command stowIntake() {
    return this.runOnce(() -> {
      this.setIntakePos(IntakeConstants.INTAKE_STOW_POS);
      this.isIntakeExtended = false;
      this.setIntakeNeutral();
      this.isIntakeWheelOn = false;
    });
  }

  public Command runIntake() {
    return this.runOnce(() -> {
      this.setIntakeVolts(IntakeConstants.INTAKING_VOLTS);
      this.isIntakeWheelOn = true;
    });
  }
  public Command runOutake() {
    return this.runOnce(() -> this.setIntakeVolts(IntakeConstants.OUTAKING_VOLTS));
  }
  public Command stopIntake() {
    return this.runOnce(() -> {
      this.setIntakeNeutral();
      isIntakeWheelOn = false;
    });
  }

  @Override
  public void periodic() {
    speedPubRoller.set(intakeRoller.getVelocity().getValueAsDouble());
    atTargetPubRoller.set(atTargetSpeedRoller());

    speedPubExtender.set(intakeExtender.getVelocity().getValueAsDouble());
    atTargetPubExtender.set(atTargetSpeedExtender());
    // This method will be called once per scheduler run
  }
}