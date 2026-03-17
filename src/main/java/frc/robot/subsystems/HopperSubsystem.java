package frc.robot.subsystems;

// | MEASUREMENTS | //
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class HopperSubsystem extends SubsystemBase {

    private boolean isRunning = false;
    private final TalonFX HopperMotor;
    private final CANrange rangeSensor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public HopperSubsystem() {
        // Initialize motors, sensors, etc. here

        HopperMotor = new TalonFX(Constants.HopperConstants.HopperMotorId);
        rangeSensor = new CANrange(Constants.HopperConstants.HopperSensorId);
        
        TalonFXConfiguration configs = new TalonFXConfiguration(); // creates configs
            configs.withMotorOutput(
                new MotorOutputConfigs()
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
                        .withKV(Constants.HopperConstants.kHopperVoltage / Constants.HopperConstants.kHopperSpeed) // 12 volts when requesting max RPS
            );

    HopperMotor.getConfigurator().apply(configs); //applies configs
    SmartDashboard.putData(this); // sends data to reading apps (dont know exactly)

        HopperMotor.getConfigurator().apply(new TalonFXConfiguration());
         } 

    private void setHopperSpeed(double speed) {
        // Code to set the speed of the hopper motor
        HopperMotor.setControl(
            velocityRequest
                .withVelocity(speed)
        );
    }

    public void runHopper() {
       // Code to deploy the hopper mechanism
        setHopperSpeed(HopperConstants.kHopperSpeed); // Example speed value
        isRunning = true;
    }

    public void stopHopper() {
        // Stop the hopper motor
        setHopperSpeed(0);
        isRunning = false;
    }

    public boolean isRunning() {
        // Return true if the hopper is running
        return isRunning;
    }
}