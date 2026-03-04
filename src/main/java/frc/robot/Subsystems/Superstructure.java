package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Globals;

import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Intake.*;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Shooter.*;

import frc.robot.tools.TunableNumber;
import frc.robot.tools.PhysicsModel;
import frc.robot.tools.ShotCalculator;
import frc.robot.tools.ShotCalculator.ShotSolution;

public class Superstructure extends SubsystemBase {
  private final SwerveSubsystem drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Kicker kicker;
  private final Agitator agitator;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private ArrayList<Translation3d> trajectoryPoint = new ArrayList<Translation3d>();
  private ArrayList<Translation3d> trajectoryVelocity = new ArrayList<Translation3d>();
  private TunableNumber manualShootRPM = new TunableNumber("Manual Shoot RPM", 2000);
  private TunableNumber manualShootHoodAngle = new TunableNumber("Manual Shoot Hood Angle", 60.0);
  private TunableNumber manualShootTurretAngle = new TunableNumber("Manual Shoot Turret Angle", 0.0);
  private ShotSolution presetShotSolution = new ShotSolution(new Rotation2d(Math.toRadians(60.0)), 2000,
      new Rotation2d(Math.PI),
      0.0, 0.0);

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
    INTAKING,
    SHOOTING,
    SHOOTING_NO_FEED,
    PASS,
    PASSING,
    ZERO,
    MANUAL_SHOOT,
    MANUAL_SHOOTING,
    PRESET_SHOOT,
    PRESET_SHOOTING,
    MANUAL_PASS, // TODO: implement ts and passing
    MANUAL_PASSING,
    
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(SwerveSubsystem drive,
      Shooter shooter, Intake intake, Kicker kicker, Agitator agitator) {
    this.drive = drive;
  
    this.shooter = shooter;
    this.intake = intake;
    this.kicker = kicker;
    this.agitator = agitator;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public void setWantedState(SuperState wantedState, ShotSolution shotSolution) {
    this.wantedSuperState = wantedState;
    this.presetShotSolution = shotSolution;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public SuperState getLastSuperState() {
    return lastState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case SHOOT:
        handleShootState();
        break;
      case SHOOTING:
        handleShootingState();
        break;
      case SHOOTING_NO_FEED:
        handleShootingNoFeedState();
        break;
      case PASS:
        handlePassState();
        break;
      case PASSING:
        handlePassingState();
        break;
      case MANUAL_SHOOT:
        handleManualShootState();
        break;
      case MANUAL_SHOOTING:
        handleManualShootingState();
        break;
      case PRESET_SHOOT:
        handlePresetShootState();
        break;
      case PRESET_SHOOTING:
        handlePresetShootingState();
        break;
      case INTAKING:
        handleIntakeingState();
        break;
      case ZERO:
        handleZeroState();
        break;
      
      default:
        handleIdleState();
        break;
    }
  }

  /**
   * This function handles the state transitions of the Superstructure subsystem.
   * It updates the current state based on the wanted state and performs necessary
   * actions.
   *
   * @return SuperState - The current state of the Superstructure subsystem after
   *         handling the state transitions.
   *
   * @param wantedSuperState The desired state of the Superstructure subsystem.
   *
   * @see SuperState
   */
  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case SHOOT:
        if (Constants.Field.isInAllianceZone(drive.getMt2Pose2d().getTranslation())) {
          if (shooter.readyToShoot() && !Constants.Field.isOnBump(drive.getMt2Pose2d().getTranslation())) {
            currentSuperState = SuperState.SHOOTING;
          } else {
            currentSuperState = SuperState.SHOOT;
          }
          break;
        }
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case PASS:
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case MANUAL_SHOOT:
        if (OI.driverA.getAsBoolean()) {
          currentSuperState = SuperState.MANUAL_SHOOTING;
        } else {
          currentSuperState = SuperState.MANUAL_SHOOT;
        }
        break;
      case MANUAL_SHOOTING:
        currentSuperState = SuperState.MANUAL_SHOOTING;
        break;
      case PRESET_SHOOT:
        if (shooter.readyToShoot()) {
          currentSuperState = SuperState.PRESET_SHOOTING;
        } else {
          currentSuperState = SuperState.PRESET_SHOOT;
        }
        break;
      case PRESET_SHOOTING:
        currentSuperState = SuperState.PRESET_SHOOTING;
        break;
      case INTAKING:
        currentSuperState = SuperState.INTAKING;
        break;
      case SHOOTING:
        currentSuperState = SuperState.SHOOTING;
        break;
      case SHOOTING_NO_FEED:
        currentSuperState = SuperState.SHOOTING_NO_FEED;
        break;
      case PASSING:
        currentSuperState = SuperState.PASSING;
        break;
      case ZERO:
        if (intake.isZeroed()) {
          intake.setWantedState(IntakeState.DOWN);
          wantedSuperState = SuperState.DEFAULT;
          currentSuperState = SuperState.DEFAULT;
        }
        currentSuperState = SuperState.ZERO;
        break;
      case MANUAL_CLIMBING:
        currentSuperState = SuperState.MANUAL_CLIMBING;
        break;
      case MANUAL_EXTEND_CLIMBER:
        currentSuperState = SuperState.MANUAL_EXTEND_CLIMBER;
        break;
      case AUTO_PREP_CLIMB:
        if (drive.hitSetPoint(drive.getClimbPrepSetpoint())
        /**
         * && climber.getClimberPosition() >
         * Constants.SetPoints.Climber.CLIMBER_L1_EXTEND_HEIGHT_INCHES
         **/
        ) {
          wantedSuperState = SuperState.AUTO_ALIGN_CLIMB;
          currentSuperState = SuperState.AUTO_ALIGN_CLIMB;
        } else {
          currentSuperState = SuperState.AUTO_PREP_CLIMB;
        }
        break;
      case AUTO_ALIGN_CLIMB:
        if (drive.hitSetPoint(drive.getClimbAlignSetpoint())) {
          if (DriverStation.isAutonomousEnabled()) {
            wantedSuperState = SuperState.AUTON_CLIMB;
            currentSuperState = SuperState.AUTON_CLIMB;
          } else {
            wantedSuperState = SuperState.AUTO_L3_CLIMB;
            currentSuperState = SuperState.AUTO_L3_CLIMB;
          }
        } else {
          currentSuperState = SuperState.AUTO_ALIGN_CLIMB;
        }
        break;
      case AUTON_CLIMB:
        currentSuperState = SuperState.AUTON_CLIMB;
        break;
      case AUTO_L3_CLIMB:
        currentSuperState = SuperState.AUTO_L3_CLIMB;
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  private void handleShootState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);

    // Feeder
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootingState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d target = Constants.Field.getHubPose();
      Translation3d initial = getTurretFieldPosition();
      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootingNoFeedState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.DEFAULT); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d target = Constants.Field.getHubPose();
      Translation3d initial = getTurretFieldPosition();
      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.DOWN, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePresetShootState() {
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        presetShotSolution);
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePresetShootingState() {
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        presetShotSolution);
    feeder.setWantedState(FeederState.FEED);
    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePassState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution = ShotCalculator.calculateFeedShot(
        new Pose2d(turret.toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getFeedTarget(turret.toTranslation2d()),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePassingState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution = ShotCalculator.calculateFeedShot(
        new Pose2d(turret.toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getFeedTarget(turret.toTranslation2d()),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter

    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT);
    // intake.setWantedState(IntakeState.DOWN, drive.getChassisSpeeds());
    shooter.setWantedState(ShooterState.DEFAULT);
    climber.setWantedState(ClimberState.IDLE);
  }

  public Translation3d getTurretFieldPosition() {
    return new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
  }

  public void handleManualShootState() { // TODO: not actually manual shooting
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation2d target = Constants.Field.getFeedTarget(drive.getMt2Pose2d().getTranslation());
    Logger.recordOutput("Shooter/feed target", target);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Rotation2d turret = Constants.Field.getFeedTarget(drive.getMt2Pose2d().getTranslation())
        .minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    Logger.recordOutput("Shooter/Manual Shoot Angle to Hub", turret.getDegrees());
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        turret, distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.DYNAMIC_INTAKING, drive.getChassisSpeeds());
  }

  public void handleManualShootingState() { // TODO: not actual manual shooting
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation2d target = Constants.Field.getFeedTarget(drive.getMt2Pose2d().getTranslation());
    Logger.recordOutput("Shooter/feed target", target);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Rotation2d turret = Constants.Field.getFeedTarget(drive.getMt2Pose2d().getTranslation())
        .minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    Logger.recordOutput("Shooter/Manual Shoot Angle to Hub", turret.getDegrees());
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        turret, distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE, drive.getChassisSpeeds());
  }

  public void handleIntakeingState() {
    intake.setWantedState(IntakeState.DYNAMIC_INTAKING, drive.getChassisSpeeds());
    feeder.setWantedState(FeederState.DEFAULT);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
    lights.setWantedState(LightsState.DEFAULT);
    shooter.setWantedState(ShooterState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN, drive.getChassisSpeeds());
    climber.setWantedState(ClimberState.IDLE);
  }

  public void handleZeroState() {
    drive.setWantedState(DriveState.DEFAULT);
    lights.setWantedState(LightsState.DEFAULT);
    intake.setWantedState(IntakeState.ZERO, drive.getChassisSpeeds());
    feeder.setWantedState(FeederState.DEFAULT);
    shooter.setWantedState(ShooterState.IDLE);
  }

  private void handleClimbingState() {
    climber.setWantedState(ClimberState.MANUAL_RETRACT);
    intake.setWantedState(IntakeState.UP, drive.getChassisSpeeds());
  }

  private void handleExtendClimberState() {
    climber.setWantedState(ClimberState.MANUAL_EXTEND);
    intake.setWantedState(IntakeState.UP, drive.getChassisSpeeds());
  }

  private void handleAutoPrepClimb() {
    drive.setWantedState(DriveState.DRIVE_TO_PRE_CLIMB);
    if (intake.getIntakePosition() > 5.0) {
      intake.setWantedState(IntakeState.UP);
    } else {
      climber.setWantedState(ClimberState.AUTON_EXTEND);
    }

    if (DriverStation.isAutonomous()) {
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(
          new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d()
              .getRotation()),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getFutureVelocity());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());

      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.FEED);
      } else {
        feeder.setWantedState(FeederState.DEFAULT);
      }
    }
  }

  private void handleAutoAlignClimb() {
    drive.setWantedState(DriveState.DRIVE_TO_ALIGN_CLIMB);
    climber.setWantedState(ClimberState.AUTON_EXTEND);
    if (DriverStation.isAutonomous()) {
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(
          new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d()
              .getRotation()),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getFutureVelocity());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.FEED);
      } else {
        feeder.setWantedState(FeederState.DEFAULT);
      }
    }
  }

  private void handleAutonClimb() {
    drive.setWantedState(DriveState.STOP);
    if (DriverStation.isAutonomous()) {
      climber.setWantedState(ClimberState.AUTON_RETRACT);
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(
          new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d()
              .getRotation()),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getFutureVelocity());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus()).addRPM(-80.0);
      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.FEED);
      } else {
        feeder.setWantedState(FeederState.DEFAULT);
      }
    } else {
      if (DriverStation.isTeleopEnabled()) {
        climber.setWantedState(ClimberState.L3_CLIMBING);
      } else {
        climber.setWantedState(ClimberState.AUTON_RETRACT);
      }
    }
  }

  private void handleAutoL3Climb() {
    drive.setWantedState(DriveState.STOP);
    climber.setWantedState(ClimberState.L3_CLIMBING);
    // if (DriverStation.isAutonomous()) {
    // ShotSolution shotSolution = ShotCalculator.calculateHubShot(
    // new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d()
    // .getRotation()),
    // Constants.Field.getHubPose().toTranslation2d(),
    // drive.getChassisSpeeds());
    // ShotSolution rotatedShotSolution = shotSolution
    // .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus()).addRPM(-80.0);
    // shooter.setWantedState(ShooterState.NORMAL_SHOOT,
    // rotatedShotSolution);
    // if (shooter.readyToShoot()) {
    // feeder.setWantedState(FeederState.FEED);
    // } else {
    // feeder.setWantedState(FeederState.IDLE);
    // }
    // }
  }

  

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/turret field pose", new Pose3d(getTurretFieldPosition(),
        new Rotation3d(drive.getMt2Pose2d().getRotation().plus(shooter.getRobotRelativeTurretAngle()))));
    PARTY();
    Rotation2d turret = Constants.Field.getHubPose().toTranslation2d().minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    shooter.passIdleTurretAngleToIdle(turret);

    currentSuperState = handleStateTransitions();
    if (RobotBase.isSimulation()) {
      for (int i = 0; i < trajectoryVelocity.size(); i++) {
        trajectoryVelocity.set(i, new Translation3d(trajectoryVelocity.get(i).getX(),
            trajectoryVelocity.get(i).getY(),
            trajectoryVelocity.get(i).getZ() - Constants.G * Globals.loopPeriodSecs));
        trajectoryPoint.set(i, trajectoryPoint.get(i).plus(trajectoryVelocity.get(i).times(Globals.loopPeriodSecs)));
        if (trajectoryPoint.get(i).getZ() < 0) {
          trajectoryPoint.remove(i);
          trajectoryVelocity.remove(i);
          i--;
        } else {
          Logger.recordOutput("Fuel/" + i, trajectoryPoint.get(i));
        }
      }
    }
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    Logger.recordOutput("States/Super State", currentSuperState);
    Logger.recordOutput("Shooter/Manual Shoot RPM", manualShootRPM.get());
    Logger.recordOutput("Shooter/Manual Shoot Hood Angle", manualShootHoodAngle.get());
    Logger.recordOutput("Shooter/Manual Shoot Turret Angle", manualShootTurretAngle.get());
    Logger.recordOutput("Shooter/Ready to Shoot", shooter.readyToShoot());
    applyStates();

  }
}