// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {
  private final TalonFX intakeRoller = new TalonFX(44);
  private final TalonFX intakeExtend = new TalonFX(45);

  private final VoltageOut intakeRollerRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage intakeExtendRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutral = new NeutralOut();

  public boolean isIntakeWheelOn = false;
  public boolean isIntakeExtended = false;
  public boolean hasIntakeHomed = false;
  
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

  public Command intakeAgitate() {
    return new SequentialCommandGroup(
      this.runOnce(() -> {
        if (isIntakeExtended) {
          this.setIntakePos(IntakeConstants.INTAKE_AGITATE_POS);
          this.isIntakeExtended = false;
        } else {
          this.setIntakePos(IntakeConstants.INTAKE_EXTEND_POS);
          this.isIntakeExtended = true;
        }
      }),
      new WaitCommand(IntakeConstants.INTAKE_AGITATE_MOVE_TIME_SEC)
    ).repeatedly().finallyDo(() -> {
      this.setIntakePos(IntakeConstants.INTAKE_EXTEND_POS);
      this.isIntakeExtended = true;
    });
  }

  public void setIntakeNeutral() {
    isIntakeWheelOn = false;
    intakeRoller.setControl(neutral);
  }

  private void setIntakePos(double pos) {
    intakeExtend.setControl(intakeExtendRequest.withPosition(pos));
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
    // This method will be called once per scheduler run
  }
}
