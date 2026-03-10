package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    // Hardware
    private final KrakenX60 flywheelMotor;
    private final KrakenX60 hoodMotor;
    private final CANcoder hoodEncoder;
    
    // State tracking
    private double targetRPS = 0;
    private boolean isHoodExtended = false;
    
    // Control parameters
    private static final double SPEED_TOLERANCE = 50; // RPM tolerance
    private static final double HOOD_TOLERANCE = 2.0; // degrees
    private static final double MAX_FLYWHEEL_RPS = 100.0; // Adjust based on your gearing

    public ShooterSubsystem() {
        // Initialize shooter hardware (motors, encoders, etc.)
        flywheelMotor = new KrakenX60(ShooterConstants.kFlywheelMotorPort);
        hoodMotor = new KrakenX60(ShooterConstants.kHoodMotorPort);
        hoodEncoder = new CANcoder(ShooterConstants.kHoodEncoderPort);
        
        // Configure motor behavior
        flywheelMotor.setInverted(false);
        hoodMotor.setInverted(false);
        
        // Initialize hood to retracted position
        isHoodExtended = false;
    }

    @Override
    public void periodic() {
        // Update shooter state and telemetry
        updateTelemetry();
    }

    public void setSpeed(double rps) {
        // Set the flywheel speed in revolutions per second
        targetRPS = Math.max(0, Math.min(rps, MAX_FLYWHEEL_RPS));
        
        // Convert RPS to motor output (0.0 to 1.0)
        double motorOutput = calculateMotorOutput(targetRPS);
        flywheelMotor.set(motorOutput);
    }

    private double calculateMotorOutput(double targetRPS) {
        // Convert target RPS to motor output percentage
        // This depends on your gearing and motor specs
        // Assuming max motor speed is around 100 RPS for Spark Max with typical gearing
        return Math.min(targetRPS / MAX_FLYWHEEL_RPS, 1.0);
    }

    public boolean isAtSpeed() {
        // Return true if the flywheel is at the target speed
        // This is a placeholder - implement with actual encoder feedback
        // For now, assume we're at speed after a short delay
        double currentRPS = getCurrentRPS();
        return Math.abs(currentRPS - targetRPS) < SPEED_TOLERANCE;
    }

    private double getCurrentRPS() {
        // Calculate current flywheel speed
        // TODO: Implement with actual encoder or motor feedback
        // For now, return target RPS as a placeholder
        return targetRPS;
    }

    public void stop() {
        // Stop the shooter motors
        setSpeed(0);
        flywheelMotor.stopMotor();
    }

    public void coast() {
        // Set the shooter motors to coast mode
        stop();
        // PWMSparkMax doesn't have explicit coast mode, but stopping achieves similar effect
    }

    public void setHoodAngle(double angle) {
        // Set the hood angle in degrees
        double angleClipped = Math.max(ShooterConstants.kMinHoodAngle, 
                                       Math.min(angle, ShooterConstants.kMaxHoodAngle));
        
        double motorOutput = calculateHoodMotorOutput(angleClipped);
        hoodMotor.set(motorOutput);
    }

    private double calculateHoodMotorOutput(double targetAngle) {
        // Calculate motor output based on encoder feedback
        double currentAngle = hoodEncoder.getAbsolutePosition() * 360; // Convert to degrees
        double angleDifference = targetAngle - currentAngle;
        
        // Simple proportional control
        double motorOutput = angleDifference * 0.01; // Adjust gain as needed
        return Math.max(-1.0, Math.min(motorOutput, 1.0)); // Clamp output to [-1, 1]
    }

    public void retractHood() {
        // Retract the hood to the passing position
        setHoodAngle(ShooterConstants.kPassingHoodAngle);
        isHoodExtended = false;
    }

    public void extendHood() {
        // Extend the hood to the shooting position
        setHoodAngle(ShooterConstants.kShootingHoodAngle);
        isHoodExtended = true;
    }

    public boolean isHoodExtended() {
        // Return true if the hood is extended to the shooting position
        return isHoodExtended;
    }

    public double getFlywheelSpeed() {
        // Return current flywheel speed in RPS
        return getCurrentRPS();
    }

    public double getHoodAngle() {
        // Return current hood angle in degrees
        return hoodEncoder.getAbsolutePosition() * 360;
    }

    public boolean isHoodAtTarget() {
        // Check if hood is at the target angle
        double currentAngle = getHoodAngle();
        double targetAngle = isHoodExtended 
            ? ShooterConstants.kShootingHoodAngle 
            : ShooterConstants.kPassingHoodAngle;
        return Math.abs(currentAngle - targetAngle) < HOOD_TOLERANCE;
    }

    private void updateTelemetry() {
        // Update SmartDashboard with telemetry data
        SmartDashboard.putNumber("Shooter/Flywheel RPS", getFlywheelSpeed());
        SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter/Hood Angle", getHoodAngle());
        SmartDashboard.putBoolean("Shooter/Hood Extended", isHoodExtended);
        SmartDashboard.putBoolean("Shooter/Hood At Target", isHoodAtTarget());
    }
}
