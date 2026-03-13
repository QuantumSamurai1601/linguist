package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;



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
import com.ctre.phoenix6.controls.VelocityVoltage;

//unsure on what these are, edit later
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class TowerSubsystem extends SubsystemBase {
    private boolean isFeeding = false;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0); // initial voltage
     private final TalonFX motor; //creates motor variable from the TalonFX class

    public TowerSubsystem() {
        // Initialize motors, sensors, etc. here
        motor = new TalonFX(Constants.TowerConstants.TowerMotorId); // gets motor from ID (id is 33)

        TalonFXConfiguration configs = new TalonFXConfiguration(); // creates configs
           configs.withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            );
           
         configs.withCurrentLimits( // limits currents prevents damage
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(50))
                .withSupplyCurrentLimitEnable(true)
    );
        configs.withSlot0(
                    new Slot0Configs()
                        .withKP(1)
                        .withKI(0)
                        .withKD(0)
                        .withKV(Constants.TowerConstants.kFeedVoltage/ Constants.TowerConstants.kFeedSpeed) // 12 volts when requesting max RPS
                );

    motor.getConfigurator().apply(configs); //applies configs
    SmartDashboard.putData(this); // sends data to reading apps (dont know exactly)
        
    }

    private void setTowerSpeed(double speed) {
        // Code to set the speed of the tower motor
        motor.setControl(
            velocityRequest
                .withVelocity(speed)
        );
    }

    public void feedToShooter() {
        // Code to start feeding a ball up into the shooter
        setTowerSpeed(TowerConstants.kFeedSpeed);
        isFeeding = true;
    }

    public void stopFeeding() {
        // Stop the tower motor
        setTowerSpeed(0);
        isFeeding = false;
    }

    public boolean isFeeding() { //getter
        // Return true if the tower is currently feeding a ball
        return isFeeding;
    }
}
