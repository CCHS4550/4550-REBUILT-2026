package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.FieldConstants;
import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.Agitator.AgitatorIOCTRE;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOCTRE;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Kicker.KickerIOCTRE;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.WantedSuperstructureState;
import frc.robot.Subsystems.Turret.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Turret.Rotation.RotationIOCTRE;
import frc.robot.Subsystems.Turret.Shooter.ShooterIOCTRE;
import frc.robot.Subsystems.Turret.Turret;
import frc.robot.Subsystems.Turret.Turret.TurretWantedState;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIOPhotonvision;

public class RobotContainer {
  private final Intake intake;
  private final Turret turret;
  private final Vision vision;
  // private final QuestNav questnav;

  private final Superstructure superstructure;
  private final SwerveSubsystem swerveSubsystem;
  private final Agitator agitator;
  private final Kicker kicker;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = config.getModuleConstants();

    swerveSubsystem =
        new SwerveSubsystem(
            new SwerveIOCTRE(config.getSwerveDrivetrainConstants(), config.getModuleConstants()),
            config.geRobotConfig(),
            controller,
            moduleConstants[0].SpeedAt12Volts,
            moduleConstants[0].SpeedAt12Volts
                / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    //  swerveSubsystem =
    //      new SwerveSubsystem(
    //          new SwerveIOCTRE(config.getSwerveDrivetrainConstants(),
    // config.getModuleConstants()),
    //          config.geRobotConfig(),
    //          controller,
    //          0.5,
    //          0.5 / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    intake = new Intake(new IntakeIOCTRE(config));
    kicker = new Kicker(new KickerIOCTRE(config));
    agitator = new Agitator(new AgitatorIOCTRE(config));
    turret =
        new Turret(
            new ElevationIOCTRE(config), new RotationIOCTRE(config), new ShooterIOCTRE(config));
    // turret = new Turret(new ElevationIOTest(), new RotationIOTest(), new ShooterIOTest());

    superstructure = new Superstructure(swerveSubsystem, intake, kicker, turret, agitator);
    // questnav =
    //     new QuestNav(swerveSubsystem, new
    // QuestNavIOQuest(config.getVisionConfigurations().get(1)));
    vision =
        new Vision(
            swerveSubsystem,
            new VisionIOPhotonvision("photonvision", config.getVisionConfigurations().get(0)));

    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> swerveSubsystem.setDesiredPoseForDriveToPoint(getLeftTrenchPose())))
        .onFalse(
            new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> swerveSubsystem.setDesiredPoseForDriveToPoint(getRightTrenchPose())))
        .onFalse(
            new InstantCommand(() -> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));

    controller
        .leftTrigger()
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ZERO_HUB, true)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));

    controller
        .rightTrigger()
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ACTIVE_DECISION, false)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));
    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ACTIVE_DECISION, true)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));

    controller
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ZERO_PASSING, true)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));

    controller
        .rightBumper()
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ACTIVE_DECISION, false)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));
    controller
        .rightBumper()
        .and(controller.leftBumper())
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.ACTIVE_DECISION, true)))
        .onFalse(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.IDLE, false)));
  }

  private Pose2d getLeftTrenchPose() {
    if (superstructure.isPassingZone(
        Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getX())) {
      return FieldConstants.getLeftNeutralZoneTrench();
    } else {
      return FieldConstants.getLeftAllianceZoneTrench();
    }
  }

  private Pose2d getRightTrenchPose() {
    if (superstructure.isPassingZone(
        Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getX())) {
      return FieldConstants.getLeftNeutralZoneTrench();
    } else {
      return FieldConstants.getLeftAllianceZoneTrench();
    }
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public void setTestTurretState() {
    turret.setWantedState(TurretWantedState.SHOOT_SCORE);
  }

  public void setTestPose() {
    swerveSubsystem.resetTranslationAndRotation(new Pose2d(14, 3, new Rotation2d()));
  }

  //   public boolean questPoseEstablished() {
  //     return questnav.questPoseEstablished();
  //   }

  public boolean isAtAutoStartingPose(Pose2d AutoStartingPose) {
    var distance =
        AutoStartingPose.getTranslation()
            .minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())
            .getNorm();
    return MathUtil.isNear(0.0, distance, 0.1);
  }

  public boolean isAtAutoStartingRotation(Rotation2d AutoStartingRotation) {
    return MathUtil.isNear(
        Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees(),
        AutoStartingRotation.getDegrees(),
        2);
  }
}
