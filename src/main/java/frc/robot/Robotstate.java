package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.stream.IntStream;

public class Robotstate {
  private static Robotstate instance;

  public static Robotstate getInstance() {
    if (instance == null) {
      instance = new Robotstate();
    }
    return instance;
  }

  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();
  private Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();
  private int[] allowedTagPoses = IntStream.range(1, 33).toArray();

  public record SwerveDriveObservation(Pose2d robotPose, ChassisSpeeds robotSpeeds) {}

  public void addPoseObservation(SwerveDriveObservation observation) {
    this.robotToFieldFromSwerveDriveOdometry = observation.robotPose;
    this.robotSpeeds = observation.robotSpeeds;
  }

  /** call in order to reset the vision subsystem to doing global pose estimation */
  public void resetAllowedTagPoses() {
    allowedTagPoses = IntStream.range(1, 33).toArray();
  }

  /**
   * call in order to set the vision subsystem to focus on a specific tag for something like
   * autoalign, not a perfect solution, but should reduce extra ambiguity added from other tags see
   * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
   */
  public synchronized void setAllowedTagPoses(int... allowedTags) {
    allowedTagPoses = allowedTags;
  }

  public Pose2d getRobotPoseFromSwerveDriveOdometry() {
    return robotToFieldFromSwerveDriveOdometry;
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return robotSpeeds;
  }

  public double getAllowedTagPosesLength() {
    return allowedTagPoses.length;
  }

  public int[] getAllowedTagPoses() {
    return allowedTagPoses;
  }

  public boolean getIfAllowedTagsSpecified() {
    return getAllowedTagPosesLength() != 32;
  }
}
