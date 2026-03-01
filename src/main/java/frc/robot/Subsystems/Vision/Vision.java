package frc.robot.Subsystems.Vision;

import static frc.robot.Constant.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  // consumer for our vision data, how all data leaves the subsystem
  private final VisionConsumer consumer;

  // array of VisionIO interfaces for the amount of cameras that we have, can be defined as real or
  // sim later
  private final VisionIO[] io;

  // array of the autologged vision inputs
  private final VisionIOInputsAutoLogged[] inputs;

  // alerts for if a camera gets disconnected
  private final Alert[] disconnectedAlerts;

  /**
   * constructor for our vision subsystem
   *
   * @param consumer takes in finalized vision date in order for other classes to use through the
   *     functional interface
   * @param io instances of VisionIO or classes implementing VisionIO
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    // Initialize io and the consumer
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    // thread safe is probably not needed, because other classes read information through the
    // consumer, and information is all written from the same periodic thread
    synchronized (inputs) {
      /** update auto logged inputs for every module */
      for (int i = 0; i < io.length; i++) {
        io[i].updateInputs(inputs[i]);
        Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      }

      // Initialize logging values for overall system
      List<Pose3d> allTagPoses = new LinkedList<>();
      List<Pose3d> allRobotPoses = new LinkedList<>();
      List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
      List<Pose3d> allRobotPosesRejected = new LinkedList<>();

      // Loop over cameras to create relevant values for each camera as well as calculate poses
      for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
        // Update disconnected alert
        disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

        // Initialize logging values per camera, not overall
        List<Pose3d> tagPoses = new LinkedList<>();
        List<Pose3d> robotPoses = new LinkedList<>();
        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();

        // Add tag poses for all relevant tages
        for (int tagId : inputs[cameraIndex].tagIds) {
          var tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // filter and send information for all observations of a camera
        for (var observation : inputs[cameraIndex].poseObservations) {
          boolean rejectPose =
              observation.tagCount() == 0 // Must have at least one tag
                  || (observation.tagCount() == 1
                      && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                  || Math.abs(observation.pose().getZ())
                      > maxZError // Must have realistic Z coordinate

                  // Must be within the field boundaries
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > FieldConstants.FIELD_LAYOUT.getFieldLength()
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > FieldConstants.FIELD_LAYOUT.getFieldWidth()

                  // must be from the allowed tags if we are doing precision vision
                  || rejectTagsFromTagAllowance(observation);

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
          if (Robotstate.getInstance().getRobotChassisSpeeds().omegaRadiansPerSecond > 1.0) {
            stdDevFactor *=
                Math.pow(
                    Robotstate.getInstance().getRobotChassisSpeeds().omegaRadiansPerSecond,
                    2.0); // increase standard deviation if the bots rotational speed is too high
          }
          double linearSpeed =
              Math.sqrt(
                  Math.pow(Robotstate.getInstance().getRobotChassisSpeeds().vxMetersPerSecond, 2.0)
                      * Math.pow(
                          Robotstate.getInstance().getRobotChassisSpeeds().vyMetersPerSecond, 2.0));
          if (linearSpeed > 2.0) {
            stdDevFactor *=
                linearSpeed; // increase standard deviation if the bots linear speed is too high
          }
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;

          // Send vision observation
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }
      // Log summary data
      Logger.recordOutput(
          "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted",
          allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected",
          allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    System.out.println();
  }

  /**
   * POORLY written method to determine if the observation used uses one of the april tags we wish
   * to use TODO: write this with less nesting
   */
  public boolean rejectTagsFromTagAllowance(PoseObservation observation) {
    if (!Robotstate.getInstance().getIfAllowedTagsSpecified()) {
      return false;
    }
    for (int i : Robotstate.getInstance().getAllowedTagPoses()) {
      for (int j = 0; j < observation.tagsUsed().length; j++) {
        if (observation.tagsUsed()[j] == i) {
          return false;
        }
      }
    }
    return true;
  }

  // only consumer is the questnav currently, but keep this information incase we need to plug
  // vision into a pose estimator later
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
