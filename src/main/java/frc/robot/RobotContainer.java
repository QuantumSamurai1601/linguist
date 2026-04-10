// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionLimelight;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SlewRateLimiter sotmAccelerationLimX = new SlewRateLimiter(0.15);
    private final SlewRateLimiter sotmAccelerationLimY = new SlewRateLimiter(0.15);
    private final SlewRateLimiter sotmAccelerationLimRot = new SlewRateLimiter(0.02);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Create all subsystems */
    private final Vision vision;
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final Tower tower = new Tower();
    public final Turret turret;

    public RobotContainer() {
        DataLogManager.start();
        NamedCommands.registerCommand("intakeextend", Commands.sequence(
            intake.extendIntake()
        ));
        NamedCommands.registerCommand("shootmfl", Commands.deadline(
            Commands.waitSeconds(4),
            hopper.runHopperShoot(),
            tower.runTowerShoot(),
            Commands.sequence(
                Commands.waitSeconds(2),
                intake.intakeAgitate()
            )
        ).finallyDo(() -> {
            hopper.stopHopper();
            tower.stopTower();
        }));
        NamedCommands.registerCommand("shootmfr", Commands.deadline(
            Commands.waitSeconds(4),
            hopper.runHopperShoot(),
            tower.runTowerShoot(),
            Commands.sequence(
                Commands.waitSeconds(2),
                intake.intakeAgitate()
            )
        ).finallyDo(() -> {
            hopper.stopHopper();
            tower.stopTower();
        }));

        autoChooser = AutoBuilder.buildAutoChooser("MFRLONG");
        SmartDashboard.putData("Auto Mode", autoChooser);

        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                new VisionLimelight(camera0Name, () -> drivetrain.getState().Pose.getRotation()),
                new VisionLimelight(camera1Name, () -> drivetrain.getState().Pose.getRotation()));

        turret = new Turret(drivetrain);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.b().onTrue(Commands.either(
            intake.stowIntake(),
            intake.extendIntake(),
            () -> intake.isIntakeExtended
        ));

        joystick.x().whileTrue(hopper.runHopperUnstuck())
            .onFalse(hopper.runOnce(hopper::stopHopper));

        joystick.y().whileTrue(Commands.runEnd(
            () -> {
                intake.runOutake();
                hopper.runHopperUnstuck();
            },
            () -> {
                intake.runIntake();
                hopper.stopHopper();
            }));          
        
        joystick.back().onTrue(turret.runOnce(turret::toggleTracking));

        joystick.start().whileTrue(intake.intakeAgitate());

        joystick.rightTrigger(0.5)
            .and(turret::canShoot)
            .whileTrue(Commands.parallel(
                hopper.runHopperShoot(),
                tower.runTowerShoot(),
                turret.run(() -> turret.setHoodPosRot()),
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(sotmAccelerationLimY.calculate(-joystick.getLeftY() * MaxSpeed * 0.5)) // Drive forward with negative Y (forward)
                        .withVelocityY(sotmAccelerationLimX.calculate(-joystick.getLeftX() * MaxSpeed * 0.5)) // Drive left with negative X (left)
                        .withRotationalRate(sotmAccelerationLimRot.calculate(-joystick.getRightX() * MaxAngularRate * 0.5)) // Drive counterclockwise with negative X (left)
                )
            ))
            .onFalse(Commands.parallel(
                hopper.runOnce(hopper::stopHopper),
                tower.runOnce(tower::stopTower),
                turret.runOnce(() -> turret.setHoodPosRot(0))
            ));

        joystick.povUp().onTrue(turret.runOnce(turret::hoodInchUp));
        joystick.povDown().onTrue(turret.runOnce(turret::hoodInchDown));

        // joystick.povUp().onTrue(turret.runOnce(() -> turret.shooterNudgeUp()));
        // joystick.povDown().onTrue(turret.runOnce(() -> turret.shooterNudgeDown()));

        // End-of-shift warning - FRC 6328
        for (int i = 1; i <= 5; i++) {
        double time = i;
        Trigger shiftAboutToEnd =
            new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
        shiftAboutToEnd
            .and(RobotModeTriggers.teleop())
            .onTrue(
                Commands.runEnd(
                        () -> joystick.setRumble(RumbleType.kRightRumble, 1.0),
                        () -> joystick.setRumble(RumbleType.kBothRumble, 0.0))
                    .withTimeout(0.25));
        }        
        
        // Reset hub shift timer when enabling - FRC 6328
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));

        // Seed Limelight while disabled, fuse when enabled
        RobotModeTriggers.disabled()
            .whileTrue(Commands.runOnce(() -> LimelightHelpers.SetIMUMode("limelight-left", 1)));
        RobotModeTriggers.disabled()
            .whileTrue(Commands.runOnce(() -> LimelightHelpers.SetIMUMode("limelight-right", 1)));
        RobotModeTriggers.autonomous()
            .onTrue(Commands.runOnce(() -> LimelightHelpers.SetIMUMode("limelight-left", 4)));
        RobotModeTriggers.autonomous()
            .onTrue(Commands.runOnce(() -> LimelightHelpers.SetIMUMode("limelight-right", 4)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // FRC 6328
    public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}