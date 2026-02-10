package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.QuestNav.QuestNavIOQuest;

public class RobotContainer {

  private final QuestNav questNav;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();

    questNav = new QuestNav(new QuestNavIOQuest(new Transform3d()));
    questNav.setPose(new Pose2d(3, 3, new Rotation2d()));
  }
}
