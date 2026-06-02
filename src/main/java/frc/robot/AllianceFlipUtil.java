// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import static frc.robot.subsystems.vision.VisionConstants.*;

public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? FIELD_LENGTH - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FIELD_WIDTH - y : y;
  }

  public static double flipX(double x) {
    return FIELD_LENGTH - x;
  }
  
  public static double flipY(double y) {
    return FIELD_WIDTH - y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }
  public static Translation2d flipX(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), translation.getY());
  }
  public static Translation2d flipY(Translation2d translation) {
    return new Translation2d(translation.getX(), flipY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static Bounds apply(Bounds bounds) {
    if (shouldFlip()) {
      return new Bounds(
          applyX(bounds.maxX()),
          applyX(bounds.minX()),
          applyY(bounds.maxY()),
          applyY(bounds.minY()));
    } else {
      return bounds;
    }
  }

  public record Bounds(double minX, double maxX, double minY, double maxY) {
    /** Whether the translation is contained within the bounds. */
    public boolean contains(Translation2d translation) {
      return translation.getX() >= minX()
          && translation.getX() <= maxX()
          && translation.getY() >= minY()
          && translation.getY() <= maxY();
    }

    /** Clamps the translation to the bounds. */
    public Translation2d clamp(Translation2d translation) {
      return new Translation2d(
          MathUtil.clamp(translation.getX(), minX(), maxX()),
          MathUtil.clamp(translation.getY(), minY(), maxY()));
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}