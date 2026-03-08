package frc.robot.Constant;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("UnusedVariable")
public class FieldConstants {

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static Pose2d scoringPoseBlue = new Pose2d(4.6, 4, new Rotation2d());
  public static Pose2d scoringPoseRed = new Pose2d(12, 4, new Rotation2d());

  public static Pose2d passingPoseBlue = new Pose2d(2.5, 4, new Rotation2d());
  public static Pose2d passingPoseRed = new Pose2d(14.5, 4, new Rotation2d());

  public static Pose2d leftTrenchBlueNeutralZone = new Pose2d(5.4, 7.4, new Rotation2d());
  public static Pose2d RightTrenchBlueNeutralZone = new Pose2d(5.4, 0.6, new Rotation2d());

  public static Pose2d leftTrenchBlueZone = new Pose2d(3.4, 7.4, new Rotation2d());
  public static Pose2d RightTrenchBlueZone = new Pose2d(3.4, 0.6, new Rotation2d());

  public static Pose2d leftTrenchRedNeutralZone = new Pose2d(11.3, 0.6, new Rotation2d());
  public static Pose2d RightTrenchRedNeutralZone = new Pose2d(11.3, 7.4, new Rotation2d());

  public static Pose2d leftTrenchRedZone = new Pose2d(13.3, 0.6, new Rotation2d());
  public static Pose2d RightTrenchRedZone = new Pose2d(13.3, 7.4, new Rotation2d());

  public static Pose2d middleStartingPoseBlue = new Pose2d();

  public static Pose2d middleStartingPoseRed = new Pose2d();

  public static final double HUB_HEIGHT = 1.903;

  public static double SCORE_HEIGHT_METERS = 0.0;

  public static double PASS_HEIGHT_METERS = 0.0;

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Blue;
  }

  public static Pose2d getScoringPose() {
    return isBlueAlliance() ? scoringPoseBlue : scoringPoseRed;
  }

  public static Pose2d getPassingPose() {
    return isBlueAlliance() ? passingPoseBlue : passingPoseRed;
  }

  public static Pose2d getMiddleStartingPose() {
    return isBlueAlliance() ? middleStartingPoseBlue : middleStartingPoseRed;
  }

  public static Pose2d getLeftNeutralZoneTrench() {
    return isBlueAlliance() ? leftTrenchBlueNeutralZone : leftTrenchRedNeutralZone;
  }

  public static Pose2d getRightNeutralZoneTrench() {
    return isBlueAlliance() ? RightTrenchBlueNeutralZone : RightTrenchRedNeutralZone;
  }

  public static Pose2d getLeftAllianceZoneTrench() {
    return isBlueAlliance() ? leftTrenchBlueZone : leftTrenchRedZone;
  }

  public static Pose2d getRightAllianceZoneTrench() {
    return isBlueAlliance() ? RightTrenchBlueZone : RightTrenchRedZone;
  }
}
