package frc.robot.Constant;

import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("UnusedVariable")
public class FieldConstants {
  // public static final AprilTagFieldLayout FIELD_LAYOUT;
  // 2026 field layout is not out yet

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Blue;
  }
}
