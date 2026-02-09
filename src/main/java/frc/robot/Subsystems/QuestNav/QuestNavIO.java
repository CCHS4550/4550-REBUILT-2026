package frc.robot.Subsystems.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

// creates a questnav interface for hardware or sim to fully define
public interface QuestNavIO {

  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class QuestNavIOInputs {

    

    public boolean QuestNavConnected = false;
    public boolean QuestNavTracking = false;
    public boolean hasEstablishedSetPose = false;

    public Transform3d robotToQuest = new Transform3d();

    public PoseFrame[] unreadPoseFrames = new PoseFrame[0];

    // NOT the same as data timestamps and should not be used to timestamp information
    public double appTimeStamp = 0.0; // returns optional

    public double latencyMS = 0;
    public int batteryPercent = 0; // returns optional
    public int frameCount = 0; // returns optional
    public int trackingLostCounter = 0; // returns optional
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(QuestNavIOInputs inputs) {}

  /** must be called periodically for questnav to function */
  public default void commandPeriodic() {}

  /**
   * the quests initial pose is set
   *
   * <p>Note that because questnav is a dead reckoning system, this must be set for it to work
   * properly
   */
  public default void setPose(Pose2d pose) {}
}
