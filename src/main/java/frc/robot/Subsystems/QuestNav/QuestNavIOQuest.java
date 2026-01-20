package frc.robot.Subsystems.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;

// physical questnav hardware
public class QuestNavIOQuest implements QuestNavIO {
  protected final QuestNav quest = new QuestNav();
  protected final Transform3d robotToQuest;
  private boolean initialPoseSet = false;

  /**
   * Creates a new QuestIOQuestNav
   *
   * @param robotToQuest The 3D position of the quest relative to the robot.
   */
  public QuestNavIOQuest(Transform3d robotToQuest) {
    this.robotToQuest = robotToQuest;
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.QuestNavConnected = quest.isConnected();
    inputs.QuestNavTracking = quest.isTracking();

    // if the quest has lost tracking, its data is essentially useless until it has been
    // reestablished,
    // so we can assume initial pose needs to be set again
    if (!quest.isTracking()) {
      initialPoseSet = false;
    }
    inputs.hasEstablishedSetPose = initialPoseSet;

    inputs.robotToQuest = this.robotToQuest;

    // looping over the array of unread poseframs this way should allow us to implement further
    // filtering in the future
    List<PoseFrame> poseFrames = new ArrayList<>();
    for (var i : quest.getAllUnreadPoseFrames()) {
      poseFrames.add(i);
    }
    inputs.unreadPoseFrames = poseFrames.toArray(new PoseFrame[0]);
    poseFrames.clear();

    // for these optional values, if for some reason the quest chooses not to return, it defaults to
    // zero
    // should probably throw a warning too
    inputs.appTimeStamp = quest.getAppTimestamp().orElse(0.0);

    inputs.latencyMS = quest.getLatency();

    inputs.batteryPercent = quest.getBatteryPercent().orElse(0);

    inputs.frameCount = quest.getFrameCount().orElse(0);

    inputs.trackingLostCounter = quest.getTrackingLostCounter().orElse(0);
  }

  @Override
  public void commandPeriodic() {
    quest.commandPeriodic();
  }

  @Override
  public void setPose(Pose2d pose) {
    // the pose must be adjusted by its position on the robot
    initialPoseSet = true;

    Pose3d tempPose = new Pose3d(pose);
    tempPose.transformBy(robotToQuest);

    pose = tempPose.toPose2d();

    quest.setPose(tempPose);
  }
}