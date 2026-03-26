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

public class Intake extends SubsystemBase {
  private final TalonFX intakeRoller = new TalonFX(44);
  private final TalonFX intakeExtend = new TalonFX(45);

  private final VoltageOut intakeRollerRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage intakeExtendRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  private final Debouncer debouncer = new Debouncer(0.1);

  public boolean isIntakeWheelOn = false;
  public boolean isIntakeExtended = false;
  public boolean hasIntakeHomed = false;
  
    public double getIntakeExtendPosition() {
    return intakeExtend.getPosition().getValueAsDouble();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
    speedPub = table.getDoubleTopic("Speed").publish();
    atTargetPub = table.getBooleanTopic("AtTarget").publish();

    setpointPub.set(TARGET_RPS);
  }

  public double getIntakeRollerPosition() {
    return intakeRoller.getPosition().getValueAsDouble();
  }

  public double getIntakeRollerVelocity() {
    return intakeRoller.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed() {
    return Math.abs(intakeRoller.getVelocity().getValueAsDouble() - IntakeConstants.TARGET_RPS) < IntakeConstants.ROLLER_VELOCITY_TOLERANCE_RPS;
  }
  
  /** Creates a new Intake. */
  public Intake() {
    intakeRoller.getConfigurator().apply(IntakeConstants.intakeRollerConfig);
    intakeExtend.getConfigurator().apply(IntakeConstants.intakeExtendConfig);

    intakeExtend.setPosition(0);
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
    intakeExtend.setControl(intakeExtendRequest.withPosition(pos));
  }

  public Command homeIntakeExtend() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        intakeExtend.getConfigurator().apply(IntakeConstants.homingConfig);
        intakeExtend.setControl(new DutyCycleOut(IntakeConstants.INTAKE_HOMING_DUTY_CYCLE_OUT));
      }),
      new WaitUntilCommand(() ->
        debouncer.calculate(intakeExtend.getStatorCurrent().getValueAsDouble() > IntakeConstants.INTAKE_HOMING_STATOR_CURRENT_THRES && Math.abs(intakeExtend.getVelocity().getValueAsDouble()) < IntakeConstants.INTAKE_HOMING_MAX_VELOCITY_THRES)
      ).withTimeout(2),
      new InstantCommand(() -> {
        intakeExtend.setControl(neutral);
        intakeExtend.setNeutralMode(NeutralModeValue.Brake);
      }),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
        intakeExtend.setPosition(0);
        hasIntakeHomed = true;
        intakeExtend.setNeutralMode(NeutralModeValue.Coast);
        intakeExtend.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
        intakeExtend.getConfigurator().apply(IntakeConstants.intakeExtendConfig);
        intakeExtend.setControl(intakeExtendRequest.withPosition(IntakeConstants.INTAKE_STOW_POS));
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
        if (intakeExtend.getPosition().getValueAsDouble() < 0.1) {
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
    speedPub.set(intakeRoller.getVelocity().getValueAsDouble());
    atTargetPub.set(atTargetSpeed());
    // This method will be called once per scheduler run
  }
}
