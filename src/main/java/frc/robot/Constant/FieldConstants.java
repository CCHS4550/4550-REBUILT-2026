package frc.robot.Constant;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("UnusedVariable")
public class FieldConstants {

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static Pose2d scoringPoseBlue = new Pose2d();
  public static Pose2d scoringPoseRed = new Pose2d();

  public static Pose2d passingPoseBlueLeft = new Pose2d();
  public static Pose2d passingPoseRedLeft = new Pose2d();
  public static Pose2d passingPoseBlueRight = new Pose2d();
  public static Pose2d passingPoseRedRight = new Pose2d();

  public static final double HUB_HEIGHT = 1.8288;

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
}
