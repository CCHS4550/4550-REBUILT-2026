package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware or sim to fully define
public interface VisionIO {

  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      int[] tagsUsed,
      double averageTagDistance,
      PoseObservationType type) {}

  // should just be Photonvision
  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION, // important
    QUESTNAV // important
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(VisionIOInputs inputs) {}
}
