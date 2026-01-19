package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

  public record SwerveDriveObservation(Pose2d robotPose, ChassisSpeeds robotSpeeds) {}

  public void addPoseObservation(SwerveDriveObservation observation) {
    this.robotToFieldFromSwerveDriveOdometry = observation.robotPose;
    this.robotSpeeds = observation.robotSpeeds;
  }

  public Pose2d getRobotPoseFromSwerveDriveOdometry() {
    return robotToFieldFromSwerveDriveOdometry;
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return robotSpeeds;
  }
}
