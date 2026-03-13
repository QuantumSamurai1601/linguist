package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class HopperSubsystem extends SubsystemBase {

    private boolean isRunning = false;
    private final TalonFX HopperMotor;
    private final CANrange rangeSensor;
    

    public HopperSubsystem() {
        // Initialize motors, sensors, etc. here

        HopperMotor = new TalonFX(34);
        rangeSensor = new CANrange(35);
        

        HopperMotor.getConfigurator().apply(new TalonFXConfiguration());
         } 

    private void setHopperSpeed(double speed) {
        // Code to set the speed of the hopper motor
        HopperMotor.set(speed);
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