// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elastic;
import frc.robot.Elastic.Notification;
import frc.robot.Elastic.NotificationLevel;
import frc.robot.subsystems.vision.VisionConstants.PoseObservationType;
import frc.robot.subsystems.vision.VisionConstants.VisionInputs;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionPhoton[] photons;
  private final VisionInputs[] inputs;
  private final Notification[] disconnectedAlerts;

  private final List<StructArrayLogEntry<Pose3d>> cameraTagPosesLogs = new ArrayList<>();
  private final List<StructArrayLogEntry<Pose3d>> cameraRobotPosesLogs = new ArrayList<>();
  private final List<StructArrayLogEntry<Pose3d>> cameraRobotPosesAcceptedLogs = new ArrayList<>();
  private final List<StructArrayLogEntry<Pose3d>> cameraRobotPosesRejectedLogs = new ArrayList<>();

  private final StructArrayLogEntry<Pose3d> summaryTagPosesLog;
  private final StructArrayLogEntry<Pose3d> summaryRobotPosesLog;
  private final StructArrayLogEntry<Pose3d> summaryRobotPosesAcceptedLog;
  private final StructArrayLogEntry<Pose3d> summaryRobotPosesRejectedLog;

  private final Timer notificationTimer = new Timer();

  public Vision(VisionConsumer consumer, VisionPhoton... photons) {
    this.consumer = consumer;
    this.photons = photons;
    
    DataLog log = DataLogManager.getLog();

    // Initialize inputs and logs
    this.inputs = new VisionInputs[photons.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionInputs();

      cameraTagPosesLogs.add(StructArrayLogEntry.create(log, "Vision/Camera" + i + "/TagPoses", Pose3d.struct));
      cameraRobotPosesLogs.add(StructArrayLogEntry.create(log, "Vision/Camera" + i + "/RobotPoses", Pose3d.struct));
      cameraRobotPosesAcceptedLogs.add(StructArrayLogEntry.create(log, "Vision/Camera" + i + "/RobotPosesAccepted", Pose3d.struct));
      cameraRobotPosesRejectedLogs.add(StructArrayLogEntry.create(log, "Vision/Camera" + i + "/RobotPosesRejected", Pose3d.struct));
    }

    summaryTagPosesLog = StructArrayLogEntry.create(log, "Vision/Summary/TagPoses", Pose3d.struct);
    summaryRobotPosesLog = StructArrayLogEntry.create(log, "Vision/Summary/RobotPoses", Pose3d.struct);
    summaryRobotPosesAcceptedLog = StructArrayLogEntry.create(log, "Vision/Summary/RobotPosesAccepted", Pose3d.struct);
    summaryRobotPosesRejectedLog = StructArrayLogEntry.create(log, "Vision/Summary/RobotPosesRejected", Pose3d.struct);

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Notification[photons.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Notification(NotificationLevel.WARNING, "CAMERA DISCONNECTED" , "Camera " + Integer.toString(i) + " is disconnected.", 4500);
    }

    notificationTimer.start();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < photons.length; i++) {
      photons[i].updateInputs(inputs[i]);
      // Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < photons.length; cameraIndex++) {
      // Update disconnected alert
      if (!inputs[cameraIndex].connected && notificationTimer.hasElapsed(5)) {
        Elastic.sendNotification(disconnectedAlerts[cameraIndex]); 
        notificationTimer.restart();
      }

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() <= 0 // Must have at least one tag
                || (observation.tagCount() >= 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      cameraTagPosesLogs.get(cameraIndex).append(tagPoses.toArray(new Pose3d[0]));
      cameraRobotPosesLogs.get(cameraIndex).append(robotPoses.toArray(new Pose3d[0]));
      cameraRobotPosesAcceptedLogs.get(cameraIndex).append(robotPosesAccepted.toArray(new Pose3d[0]));
      cameraRobotPosesRejectedLogs.get(cameraIndex).append(robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    summaryTagPosesLog.append(allTagPoses.toArray(new Pose3d[0]));
    summaryRobotPosesLog.append(allRobotPoses.toArray(new Pose3d[0]));
    summaryRobotPosesAcceptedLog.append(allRobotPosesAccepted.toArray(new Pose3d[0]));
    summaryRobotPosesRejectedLog.append(allRobotPosesRejected.toArray(new Pose3d[0]));    
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}