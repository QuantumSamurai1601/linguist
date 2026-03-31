// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
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
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

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
            new WaitCommand(4),
            hopper.runHopperShoot(),
            tower.runTowerShoot()
        ).finallyDo(() -> {
            hopper.stopHopper();
            tower.stopTower();
        }));
        NamedCommands.registerCommand("shootmfr", Commands.deadline(
            new WaitCommand(4),
            hopper.runHopperShoot(),
            tower.runTowerShoot()
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

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );

        // joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.b().onTrue(Commands.either(
            intake.stowIntake(),
            intake.extendIntake(),
            () -> intake.isIntakeExtended
        ));

        joystick.x().whileTrue(hopper.runHopperUnstuck());
        joystick.x().onFalse(hopper.runOnce(() -> hopper.stopHopper()));

        joystick.back().onTrue(turret.runOnce(() -> turret.toggleTracking()));        

        joystick.rightTrigger(0.5).onTrue(Commands.parallel(
            hopper.runHopperShoot(),
            tower.runTowerShoot()
            // turret.turretShoot()
        ));

        joystick.rightTrigger(0.5).onFalse(Commands.parallel(
            hopper.runOnce(() -> hopper.stopHopper()),
            tower.runOnce(() -> tower.stopTower())
            // turret.turretShoot()
        ));

        joystick.povUp().onTrue(turret.runOnce(() -> turret.hoodInchUp()));

        joystick.povDown().onTrue(turret.runOnce(() -> turret.hoodInchDown()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
