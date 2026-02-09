package frc.robot.Subsystems.QuestNav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Vision.Vision;
import gg.questnav.questnav.PoseFrame;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// this class listens to Vision and robot state for occasionally forced pose resets
// this class implements drive as a listener to periodically update
public class QuestNavSubsystem extends SubsystemBase implements Vision.VisionConsumer {
  private final QuestConsumer consumer;

  // logging and interaction with the hardware
  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  private final Alert disconnectedAlert;
  private final Alert noTrackingAlert;

  // timer to be used to determine when to forcibly reset to robot pose
  private final Timer timer = new Timer();

  // because the questnav is such a robust odometry system, these standard deviations realistically
  // should not need to change.
  private final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  public QuestNavSubsystem(QuestConsumer consumer, QuestNavIO io) {
    this.consumer = consumer;
    this.io = io;

    disconnectedAlert = new Alert("The quest is disconnected", AlertType.kWarning);
    noTrackingAlert = new Alert("The quest is not tracking", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // unsure whether or not this should be synchronized or not
    io.commandPeriodic();
    synchronized (inputs) {
      io.updateInputs(inputs);
      Logger.processInputs("Questnav", inputs);

      List<Pose3d> allPoseFrames = new LinkedList<>();
      List<Pose3d> rejectedPoseFrames = new LinkedList<>();
      List<Pose3d> acceptedPoseFrames = new LinkedList<>();

      // supposed to loop through all available pose frames and mark if they ought to be used
      // the official implementation simply reads the most recent frame, which may benefit on field
      // performance
      for (PoseFrame givenPoseFrame : inputs.unreadPoseFrames) {
        boolean rejectFrame = !inputs.QuestNavTracking || !inputs.hasEstablishedSetPose;

        allPoseFrames.add(givenPoseFrame.questPose3d());
        if (rejectFrame) {
          rejectedPoseFrames.add(givenPoseFrame.questPose3d());
        } else {
          acceptedPoseFrames.add(givenPoseFrame.questPose3d());

          Pose3d transformedPose =
              givenPoseFrame.questPose3d().transformBy(inputs.robotToQuest.inverse());

          consumer.accept(
              transformedPose.toPose2d(), givenPoseFrame.dataTimestamp(), QUESTNAV_STD_DEVS);
        }
      }

      Logger.recordOutput("Questnav", allPoseFrames.toArray(new Pose2d[allPoseFrames.size()]));
      Logger.recordOutput(
          "Questnav", rejectedPoseFrames.toArray(new Pose2d[rejectedPoseFrames.size()]));
      Logger.recordOutput(
          "Questnav", acceptedPoseFrames.toArray(new Pose2d[acceptedPoseFrames.size()]));

      backupOdometrySetter();

      disconnectedAlert.set(!inputs.QuestNavConnected);
      noTrackingAlert.set(!inputs.QuestNavTracking);
    }
  }

  public synchronized void setPose(
      Pose2d
          pose) { // needs to be synchronized b/c vision & the backup odometry could hypothetically
    // try to update the pose at same time
    io.setPose(pose);
  }

  /**
   * if the questnav has gone long enough (5 arbitrary seconds) and vision has not given us a proper
   * pose to reset to then we reset to the robot pose and hope for the best to at least reestablish
   * quest tracking
   */
  private void backupOdometrySetter() {
    if (!inputs.hasEstablishedSetPose && !timer.isRunning()) {
      timer.reset();
      timer.start();
    }

    if (timer.hasElapsed(5)) {
      setPose(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry());
      timer.stop();
      timer.reset();
    }
  }

  @FunctionalInterface
  public interface QuestConsumer {
    void accept(
        Pose2d questRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> questMeasurementStdDevs);
  }

  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Pose3d transformedPose = new Pose3d(visionRobotPoseMeters).transformBy(inputs.robotToQuest);

    setPose(transformedPose.toPose2d());
  }
}
