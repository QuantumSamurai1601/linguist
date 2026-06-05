// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.AllianceFlipUtil;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static double FIELD_LENGTH = aprilTagLayout.getFieldLength();
  public static double FIELD_WIDTH = aprilTagLayout.getFieldWidth();

  // Trench shenanigans pray to god...
  // Robot center w/ intake extended is 21.75 which is half of 43.5
  // 35 (Bot Width) + 11 (Intake Extend) - 3.75 (1 Bumper) + 1.5 (Tolerance) = 43.5

  // .        . (X1, Y2) (X2,Y2)
  //
  //
  // .        . (X1, Y1) (X2, Y1)
  public static double BLUE_TRENCH_X1 = Units.inchesToMeters(134.06); // Blue trench tape X line at 156.06 - 18 (1/2 Robot) - 4
  public static double BLUE_TRENCH_X2 = Units.inchesToMeters(229.06); // Blue trench neutral X line at 207.06 + 18 + 4
  public static double RED_TRENCH_X1 = AllianceFlipUtil.flipX(BLUE_TRENCH_X1);
  public static double RED_TRENCH_X2 = AllianceFlipUtil.flipX(BLUE_TRENCH_X2);
  public static double TRENCH_Y_BOTTOM1 = Units.inchesToMeters(14); // Blue trench wall Y at 0 + 18 - 4
  public static double TRENCH_Y_BOTTOM2 = Units.inchesToMeters(35.82); // Blue trench triangle Y at 49.82 - 18 + 4

  public static Translation2d BLUE_TRENCH_BL_COR = new Translation2d(BLUE_TRENCH_X1, TRENCH_Y_BOTTOM1); // Bottom left
  public static Translation2d BLUE_TRENCH_TR_COR = new Translation2d(BLUE_TRENCH_X2, TRENCH_Y_BOTTOM2); // Top right
  public static Translation2d RED_TRENCH_BR_COR = new Translation2d(RED_TRENCH_X1, TRENCH_Y_BOTTOM1); // Bottom right
  public static Translation2d RED_TRENCH_TL_COR = new Translation2d(RED_TRENCH_X2, TRENCH_Y_BOTTOM2); // Top left

  public static Rectangle2d BLUE_BOT_TRENCH = new Rectangle2d(BLUE_TRENCH_BL_COR, BLUE_TRENCH_TR_COR);
  public static Rectangle2d BLUE_TOP_TRENCH = new Rectangle2d(AllianceFlipUtil.flipY(BLUE_TRENCH_BL_COR), AllianceFlipUtil.flipY(BLUE_TRENCH_TR_COR)); // (TL, BR)
  public static Rectangle2d RED_BOT_TRENCH = new Rectangle2d(RED_TRENCH_BR_COR, RED_TRENCH_TL_COR);
  public static Rectangle2d RED_TOP_TRENCH = new Rectangle2d(AllianceFlipUtil.flipY(RED_TRENCH_BR_COR), AllianceFlipUtil.flipY(RED_TRENCH_TL_COR)); // (TR, BL)

  public static double TRENCH_ALIGN_X_LOOKAHEAD_SEC = 0.5;
  public static double TRENCH_ALIGN_INTAKE_RETRACT_DEG = 20;

  public static Pose2d getClosestTrenchPose(Pose2d current) {
    Rectangle2d[] trenches = {
        BLUE_BOT_TRENCH,
        BLUE_TOP_TRENCH,
        RED_BOT_TRENCH,
        RED_TOP_TRENCH
        };
    Rectangle2d closestTrench = trenches[0];
    double closestDistance = closestTrench.getDistance(current.getTranslation());
    Rotation2d closestRotation = Math.abs(current.getRotation().getDegrees()) <= (90) ? Rotation2d.kZero : Rotation2d.k180deg;

    for (int i = 1; i < trenches.length; i++) {
      double distance = trenches[i].getDistance(current.getTranslation());
      if (distance < closestDistance) {
        closestTrench = trenches[i];
        closestDistance = distance;
      }
    }

    return new Pose2d(closestTrench.getCenter().getTranslation(), closestRotation);
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "photoncam-left"; // 10.16.1.11
  public static String camera1Name = "photoncam-right"; // 10.16.1.12
  public static String camera2Name = "photoncam-turret"; // 10.16.1.13

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(Units.inchesToMeters(11.618), Units.inchesToMeters(9.465), Units.inchesToMeters(8.716),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(125)));
  public static Transform3d robotToCamera1 =
      new Transform3d(Units.inchesToMeters(11.618), Units.inchesToMeters(-9.465), Units.inchesToMeters(8.716),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(-125)));
  public static Transform3d robotToCamera2 =
      new Transform3d(Units.inchesToMeters(10.377), Units.inchesToMeters(-7.725), Units.inchesToMeters(20.416), 
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-52.6), Units.degreesToRadians(37.5)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.1;
  public static double maxTagDistance = Units.inchesToMeters(66.0);
  public static double maxTagArea = 0.06;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = 
    Double.POSITIVE_INFINITY; // No rotation data available

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public static class VisionInputs {
    public boolean connected = false;
    // public TargetObservation latestTargetObservation =
    //     new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }
}