package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = config.getModuleConstants();

    // swerveSubsystem =
    //     new SwerveSubsystem(
    //         new SwerveIOCTRE(config.getSwerveDrivetrainConstants(), config.getModuleConstants()),
    //         config.geRobotConfig(),
    //         controller,
    //         moduleConstants[0].SpeedAt12Volts,
    //         moduleConstants[0].SpeedAt12Volts
    //             / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    swerveSubsystem =
        new SwerveSubsystem(
            new SwerveIOCTRE(config.getSwerveDrivetrainConstants(), config.getModuleConstants()),
            config.geRobotConfig(),
            controller,
            0.5,
            0.5 / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(90))));
    controller
        .a()
        .whileTrue(
            new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.ROTATION_LOCK)));
    controller
        .a()
        .whileFalse(
            new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }
}
