package frc.robot.Config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConfig {
  public enum VisionType {
    PHOTONVISION,
    QUESTNAV,
    NONE
  }

  private final String name;

  public double visionMountingRollRadians = 0;
  public double visionMountingYawRadians = 0;
  public double visionMountingPitchRadians = 0;
  public double visionHeightOffsetMeters = 0;
  public double visionLengthOffsetMeters = 0;
  public double visionWidthOffsetMeters = 0;
  public VisionType visionType = VisionType.NONE;

  public double visionDistanceScalarValue = 0;

  public VisionConfig(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public VisionConfig withMountingRoll(double rollRadians) {
    this.visionMountingRollRadians = rollRadians;
    return this;
  }

  public VisionConfig withMountingYaw(double yawRadians) {
    this.visionMountingYawRadians = yawRadians;
    return this;
  }

  public VisionConfig withMountingPitch(double pitchRadians) {
    this.visionMountingPitchRadians = pitchRadians;
    return this;
  }

  public VisionConfig withHeightOffset(double heightOffsetMeters) {
    this.visionHeightOffsetMeters = heightOffsetMeters;
    return this;
  }

  public VisionConfig withLengthOffset(double lengthOffsetMeters) {
    this.visionLengthOffsetMeters = lengthOffsetMeters;
    return this;
  }

  public VisionConfig withWidthOffset(double widthOffsetMeters) {
    this.visionWidthOffsetMeters = widthOffsetMeters;
    return this;
  }

  public VisionConfig withDistanceScalar(double distanceScalar) {
    this.visionDistanceScalarValue = distanceScalar;
    return this;
  }

  public Translation3d getTranslationOffset() {
    return new Translation3d(
        this.visionLengthOffsetMeters, this.visionHeightOffsetMeters, this.visionWidthOffsetMeters);
  }

  public Rotation3d getRotationOffset() {
    return new Rotation3d(
        this.visionMountingRollRadians,
        this.visionMountingPitchRadians,
        this.visionMountingYawRadians);
  }

  public Translation2d getTranslationToRobotCenter() {
    return new Translation2d(this.visionLengthOffsetMeters, this.visionWidthOffsetMeters);
  }

  public VisionType getType() {
    return visionType;
  }
}
