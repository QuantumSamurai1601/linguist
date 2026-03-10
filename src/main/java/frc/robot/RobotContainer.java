// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


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

    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();

    public final VisionSubsystem vision = new VisionSubsystem();

    private ScoringMode scoringMode = ScoringMode.SHOOTING;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

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

        /*** Controller mappings ***/

        // ── LT – Hopper roller toggle ON / OFF ────────────────────────────────────
        // The roller silently does nothing if the arm is not extended.
        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> {
                if (hopper.isRunning()) {
                    hopper.stopHopper();
                } else {
                    hopper.runHopper();   // HopperSubsystem needs to guard against Intake retracted state
                }
            })
        );

        // ── RT – Shoot sequence (hold) ────────────────────────────────────────────
        // 1. Spin flywheel to the speed for the current mode (SHOOTING or PASSING).
        // 2. Block until flywheel reports isAtSpeed().
        // 3. Feed fuel from tower up into the shooter.
        // 4. On RT release → whileTrue interrupts → finallyDo coasts + stops feed.
        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                // Branch A: keep flywheel at the right speed for the whole hold
                Commands.run(() -> shooter.setSpeed(
                    scoringMode == ScoringMode.SHOOTING
                        ? ShooterConstants.kShootingRPS
                        : ShooterConstants.kPassingRPS
                ), shooter),
                // Branch B: wait for speed, then feed
                Commands.waitUntil(shooter::isAtSpeed)
                        .andThen(Commands.run(() -> tower.feedToShooter(), tower))
            ).finallyDo((interrupted) -> {
                shooter.coast();
                tower.stopFeeding();
            })
        );

        // ── A – Intake arm extend / retract toggle ────────────────────────────────
        // Retracting also stops the roller automatically.
        joystick.a().onTrue(
            Commands.runOnce(() -> {
                if (intake.isArmExtended()) {
                    intake.retractArm();   // retractArm() calls stopIntake() internally
                } else {
                    intake.extendArm();
                }
            })
        );

        // ── Y – Toggle Shooting ↔ Passing mode ────────────────────────────────────
        // Shooting mode → full flywheel speed, hood at shooting angle, aims at hub.
        // Passing mode → lower flywheel speed, hood at loft angle, aims at trench tags.
        joystick.y().onTrue(
            Commands.runOnce(() -> {
                if (scoringMode == ScoringMode.SHOOTING) {
                    scoringMode = ScoringMode.PASSING;
                } else {
                    scoringMode = ScoringMode.SHOOTING;
                }
            })
        );

        // ── X – Manual hood override ──────────────────────────────────────────────
        // If extended → retract. If retracted → extend
        joystick.x().onTrue(
            Commands.runOnce(() -> {
                if (shooter.isHoodExtended()) {
                    shooter.retractHood();
                } else {
                    shooter.extendHood();
                }
            })
        );

        // ── Auto-retract hood near the trench ────────────────────────────────────
        // Logic-driven trigger — no button press needed. Fires whenever the robot
        // enters the trench proximity zone (odometry or vision). Restores the
        // mode-appropriate angle when the robot leaves.
        new Trigger(this::isNearTrench)
            .onTrue(Commands.runOnce(() -> {
                scoringMode = ScoringMode.PASSING;
                shooter.retractHood();
            }))
            .onFalse(Commands.runOnce(() -> {
                scoringMode = ScoringMode.SHOOTING;
                shooter.extendHood();
            }));

        // ── LB – X-lock wheels (defensive) ──────────────────────────────────────
        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // ── RB – Re-seed field-centric heading ────────────────────────────────
        joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Hold the D-pad up/down to drive straight forward/backward at 0.5 m/s
        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // Hold back + Y to run the forward dynamic routine, which will sweep the robot forward through a range of speeds.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // Hold back + X to run the reverse dynamic routine, which will sweep the robot backward through a range of speeds.
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Hold start + Y to run the forward quasistatic routine, which will slowly increase the robot's speed until it reaches max speed.
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // Hold start + X to run the reverse quasistatic routine, which will slowly increase the robot's speed until it reaches max speed.
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private boolean isNearTrench() {
        // Could use odometry to check the robot's position
        return false;
    }

 }
