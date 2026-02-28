package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Intake.IntakeIOCTRE;
import frc.robot.Subsystems.Turret.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Turret.Rotation.RotationIOCTRE;
import frc.robot.Subsystems.Turret.Shooter.ShooterIOCTRE;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final Intake intake;
  private final Turret turret;

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
    intake = new Intake(new IntakeIOCTRE(config));
    turret =
        new Turret(
            new ElevationIOCTRE(config), new RotationIOCTRE(config), new ShooterIOCTRE(config));

    // controller
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(90))));
    // controller
    //     .a()
    //     .whileTrue(
    //         new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.ROTATION_LOCK)));
    // controller
    //     .a()
    //     .whileFalse(
    //         new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));

    // controller
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 swerveSubsystem.setDesiredPoseForDriveToPointWithConstraints(
    //                     new Pose2d(0.5, 0.5, new Rotation2d(Units.degreesToRadians(67))),
    //                     1,
    //                     3.14)));
    // controller
    //     .b()
    //     .whileTrue(
    //         new InstantCommand(() ->
    // swerveSubsystem.setWantedState(WantedState.DRIVE_TO_POINT)));

    // controller
    //     .b()
    //     .whileFalse(
    //         new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));
    controller
        .x()
        .whileTrue(
            new InstantCommand(
                () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE)));
    controller
        .b()
        .whileFalse(
            new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));
    // controller
    //     .x()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE)));
    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING)));
    // controller
    //     .rightTrigger()
    //     .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.IDLE)));

    controller
        .y()
        .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.STOWED)));

    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 turret.setWantedTurretMeasurables(
    //                     new TurretMeasurables(
    //                         new Rotation2d(Units.degreesToRadians(20)),
    //                         new Rotation2d(Units.degreesToRadians(20), 2)))));

    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 turret.setWantedTurretMeasurables(
    //                     new TurretMeasurables(
    //                         new Rotation2d(0), new Rotation2d(45 * ((2 * Math.PI) / 360))))));

    controller
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> turret.setWantedState(TurretWantedState.TESTING)));

    controller
        .rightTrigger()
        .onFalse(new InstantCommand(() -> turret.setWantedState(TurretWantedState.IDLE)));

    controller
        .rightTrigger()
        .whileFalse(new InstantCommand(() -> turret.setWantedState(TurretWantedState.IDLE)));
    // controller
    //     .y()
    //     .whileTrue(new InstantCommand(() ->
    // intake.setWantedIntakeState(WantedIntakeState.STOWED)));
  }

    public SwerveSubsystem getSwerveSubsystem() {
      return swerveSubsystem;
    }
}
