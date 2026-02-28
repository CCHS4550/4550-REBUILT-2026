package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constant.Constants;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Util.SubsystemDataProcessor;
import frc.robot.Util.SysIdMechanism;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase implements QuestNav.QuestConsumer {
  private final PIDController choreoXController = new PIDController(1.14, 0, 0);
  private final PIDController choreoYController = new PIDController(1.14, 0, 0);
  private final PIDController choreoThetaController = new PIDController(0.5, 0, 0);

  private Trajectory<SwerveSample> desiredChoreoTrajectory;
  private final Timer choreoTimer = new Timer();
  private Optional<SwerveSample> choreoSampleToBeApplied;

  private static final double CONTROLLER_DEADBAND = 0.1;
  public static final double TRANSLATION_ERROR_MARGIN_METERS_DRIVE_TO_POINT =
      Units.inchesToMeters(1.0);
  public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;

  public static final double ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK_DEGREES = 10.0;

  public double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

  private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);
  private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private static final double SKEW_COMPENSATION_SCALAR = -0.03;

  public enum WantedState {
    SYS_ID,
    TELEOP_DRIVE,
    CHOREO_PATH,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    IDLE
  }

  public enum SystemState {
    SYS_ID,
    TELEOP_DRIVE,
    CHOREO_PATH,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    IDLE
  }

  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;

  @AutoLogOutput private Rotation2d desiredRotationForRotationLockState;

  @AutoLogOutput private Pose2d desiredPoseForDriveToPoint = new Pose2d();

  final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

  private final Object moduleIOLock = new Object();

  private SwerveIO io = new SwerveIO() {};
  private final CommandXboxController controller;

  private final double maxVelocity;
  private final double maxAngularVelocity;

  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;
  private double maximumAngularVelocityForDriveToPoint = Double.NaN;

  private SysIdRoutine wheelRadiuSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(5),
              Seconds.of(5),
              (state) -> Logger.recordOutput("SysID Wheel Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> {
                io.setSwerveState(translationCharacterization.withVolts(volts));
              },
              log -> {
                double omega = 0.0;

                omega +=
                    ((frontLeftInputs.driveVelocityRotationsPerSecond * Math.PI * 2)
                            / Constants.SysIdConstants.DRIVE_GEAR_RATIO)
                        * 2.0
                        * Math.PI;
                omega +=
                    ((frontRightInputs.driveVelocityRotationsPerSecond * Math.PI * 2)
                            / Constants.SysIdConstants.DRIVE_GEAR_RATIO)
                        * 2.0
                        * Math.PI;
                omega +=
                    ((backLeftInputs.driveVelocityRotationsPerSecond * Math.PI * 2)
                            / Constants.SysIdConstants.DRIVE_GEAR_RATIO)
                        * 2.0
                        * Math.PI;
                omega +=
                    ((backRightInputs.driveVelocityRotationsPerSecond * Math.PI * 2)
                            / Constants.SysIdConstants.DRIVE_GEAR_RATIO)
                        * 2.0
                        * Math.PI;

                omega /= 4.0;

                ChassisSpeeds speeds = swerveInputs.Speeds;
                double v = speeds.vxMetersPerSecond;

                double wheelRadius = v / omega;
                Logger.recordOutput("SysID/WheelRadiusMeters", wheelRadius);
              },
              this));

  private final SysIdRoutine translationSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Constants.SysIdConstants.TRANSLATION_RAMP_RATE,
              Constants.SysIdConstants.TRANSLATION_STEP_RATE,
              Constants.SysIdConstants.TRANSLATION_TIMEOUT,
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setSwerveState(translationCharacterization.withVolts(output)),
              null,
              this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   */
  private final SysIdRoutine rotationSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Constants.SysIdConstants.ROTATION_RAMP_RATE,
              Constants.SysIdConstants.ROTATION_STEP_RATE,
              Constants.SysIdConstants.ROTATION_TIMEOUT,
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports
                "volts" */

                io.setSwerveState(rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private final SysIdRoutine steerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Constants.SysIdConstants.STEER_RAMP_RATE,
              Constants.SysIdConstants.STEER_STEP_RATE,
              Constants.SysIdConstants.STEER_TIMEOUT,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> io.setSwerveState(steerCharacterization.withVolts(volts)), null, this));

  public SwerveSubsystem(
      SwerveIO io,
      RobotConfig PP_Config,
      CommandXboxController controller,
      double maxVelocity,
      double maxAngularVelocity) {
    this.io = io;
    this.controller = controller;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    // io.registerTelemetryFunction(swerveInputs);
    driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);

    driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
        () -> {
          synchronized (moduleIOLock) {
            io.updateModuleInputs(
                frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
          }
        },
        io);
  }

  /**
   * Runs the quasi-static SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the quasi-static SysId test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
    final SysIdRoutine routine =
        switch (mechanism) {
          case SWERVE_TRANSLATION -> translationSysIdRoutine;
          case SWERVE_ROTATION -> rotationSysIdRoutine;
          case SWERVE_STEER -> steerSysIdRoutine;
          default -> throw new IllegalArgumentException(
              String.format("Mechanism %s is not supported.", mechanism));
        };

    return routine.quasistatic(direction);
  }

  /**
   * Runs the dynamic SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the dynamic SysId test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
    final SysIdRoutine routine =
        switch (mechanism) {
          case SWERVE_TRANSLATION -> translationSysIdRoutine;
          case SWERVE_ROTATION -> rotationSysIdRoutine;
          case SWERVE_STEER -> steerSysIdRoutine;
          default -> throw new IllegalArgumentException(
              String.format("Mechanism %s is not supported.", mechanism));
        };

    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateSwerveInputs(swerveInputs);
    synchronized (moduleIOLock) {
      Logger.processInputs("Subsystems/Drive", swerveInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Left", frontLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Right", frontRightInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Left", backLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Right", backRightInputs);
    }

    systemState = handleStateTransition();

    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);
    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;

      case CHOREO_PATH -> {
        if (systemState != SystemState.CHOREO_PATH) {
          choreoTimer.restart();
          choreoSampleToBeApplied = desiredChoreoTrajectory.sampleAt(choreoTimer.get(), false);
          yield SystemState.CHOREO_PATH;
        } else {
          choreoSampleToBeApplied = desiredChoreoTrajectory.sampleAt(choreoTimer.get(), false);
          yield SystemState.CHOREO_PATH;
        }
      }

      case ROTATION_LOCK -> SystemState.ROTATION_LOCK;
      case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) { // case something something the very basic python things -- shirly 2026
        // (i was held hostage mb)
      default:
      case SYS_ID:
        break;
      case TELEOP_DRIVE:
        io.setSwerveState(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        break;
      case CHOREO_PATH:
        {
          if (choreoSampleToBeApplied.isPresent()) {
            var sample = choreoSampleToBeApplied.get();
            Logger.recordOutput("Subsystems/Drive/Choreo/Timer Value", choreoTimer.get());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/Traj Name", desiredChoreoTrajectory.name());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/Total time", desiredChoreoTrajectory.getTotalTime());
            Logger.recordOutput("Subsystems/Drive/Choreo/sample/Desired Pose", sample.getPose());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Module Forces X", sample.moduleForcesX());
            Logger.recordOutput(
                "Subsystems/Drive/Choreo/sample/Module Forces Y", sample.moduleForcesY());
            synchronized (swerveInputs) {
              var pose = swerveInputs.Pose;

              var targetSpeeds = sample.getChassisSpeeds();
              targetSpeeds.vxMetersPerSecond += choreoXController.calculate(pose.getX(), sample.x);
              targetSpeeds.vyMetersPerSecond += choreoYController.calculate(pose.getY(), sample.y);
              targetSpeeds.omegaRadiansPerSecond +=
                  choreoThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

              io.setSwerveState(
                  new SwerveRequest.ApplyFieldSpeeds()
                      .withSpeeds(targetSpeeds)
                      .withWheelForceFeedforwardsX(sample.moduleForcesX())
                      .withWheelForceFeedforwardsY(sample.moduleForcesY())
                      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
            }
          }
          break;
        }
      case ROTATION_LOCK:
        double error =
            MathUtil.angleModulus(
                desiredRotationForRotationLockState
                    .minus(swerveInputs.Pose.getRotation())
                    .getRadians());
        System.out.println(error);

        double ff = 0.5; // example
        double ffDirection = Math.signum(error) * ff;

        double kP = 0.3;
        if (error > 3.075 || error < -3.075) {
          kP = 0.0;
          ffDirection = 0.0;
        }
        io.setSwerveState(
            driveAtAngle
                .withVelocityX(calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
                .withVelocityY(calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
                .withTargetDirection(desiredRotationForRotationLockState)
                .withHeadingPID(kP, 0.0, 0.00)
                .withTargetRateFeedforward(ffDirection)
                .withMaxAbsRotationalRate(3.14));
        break;
      case DRIVE_TO_POINT:
        var translationToDesiredPoint =
            desiredPoseForDriveToPoint.getTranslation().minus(swerveInputs.Pose.getTranslation());
        var linearDistance = translationToDesiredPoint.getNorm();
        var frictionConstant = 0.0;
        if (linearDistance >= Units.inchesToMeters(0.5)) {
          frictionConstant = DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * maxVelocity;
        }
        var directionOfTravel = translationToDesiredPoint.getAngle();
        var velocityOutput = 0.0;
        if (DriverStation.isAutonomous()) {
          velocityOutput =
              Math.min(
                  Math.abs(autoDriveToPointController.calculate(linearDistance, 0))
                      + frictionConstant,
                  maxVelocityOutputForDriveToPoint);
        } else {
          velocityOutput =
              Math.min(
                  Math.abs(teleopDriveToPointController.calculate(linearDistance, 0))
                      + frictionConstant,
                  maxVelocityOutputForDriveToPoint);
        }
        var xComponent = velocityOutput * directionOfTravel.getCos();
        var yComponent = velocityOutput * directionOfTravel.getSin();

        Logger.recordOutput("Subsystems/Drive/DriveToPoint/xVelocitySetpoint", xComponent);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/yVelocitySetpoint", yComponent);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/velocityOutput", velocityOutput);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/linearDistance", linearDistance);
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/directionOfTravel", directionOfTravel);
        Logger.recordOutput(
            "Subsystems/Drive/DriveToPoint/desiredPoint", desiredPoseForDriveToPoint);

        Rotation2d desiredRotFlipped180 =
            desiredPoseForDriveToPoint.getRotation().plus(Rotation2d.k180deg);

        if (Double.isNaN(maximumAngularVelocityForDriveToPoint)) {
          error =
              MathUtil.angleModulus(
                  desiredRotFlipped180.minus(swerveInputs.Pose.getRotation()).getRadians());
          System.out.println(error);

          ff = 0.5; // example
          ffDirection = Math.signum(error) * ff;

          kP = 0.3;
          if (error > 3.075 || error < -3.075) {
            kP = 0.0;
            ffDirection = 0.0;
          }
          io.setSwerveState(
              driveAtAngle
                  .withVelocityX(xComponent)
                  .withVelocityY(yComponent)
                  .withTargetDirection(desiredRotFlipped180)
                  .withHeadingPID(kP, 0.0, 0.00)
                  .withTargetRateFeedforward(ffDirection));
        } else {
          error =
              MathUtil.angleModulus(
                  desiredRotFlipped180.minus(swerveInputs.Pose.getRotation()).getRadians());
          System.out.println(error);

          ff = 0.5; // example
          ffDirection = Math.signum(error) * ff;

          kP = 0.3;
          if (error > 3.075 || error < -3.075) {
            kP = 0.0;
            ffDirection = 0.0;
          }
          io.setSwerveState(
              driveAtAngle
                  .withVelocityX(xComponent)
                  .withVelocityY(yComponent)
                  .withTargetDirection(desiredRotFlipped180)
                  .withHeadingPID(kP, 0.0, 0.00)
                  .withTargetRateFeedforward(ffDirection)
                  .withMaxAbsRotationalRate(maximumAngularVelocityForDriveToPoint));
        }
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    synchronized (moduleIOLock) {
      io.updateSimState();
    }
  }

  public void setState(WantedState state) {
    this.wantedState = state;
  }

  public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
    this.desiredChoreoTrajectory = trajectory;
    this.wantedState = WantedState.CHOREO_PATH;
    choreoTimer.reset();
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = Double.NaN;
  }

  public void setDesiredPoseForDriveToPointWithMaximumAngularVelocity(
      Pose2d pose, double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPointWithConstraints(
      Pose2d pose,
      double maxVelocityOutputForDriveToPoint,
      double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND);
    double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND);

    xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
    yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    double xVelocity =
        (FieldConstants.isBlueAlliance() ? -xMagnitude * maxVelocity : xMagnitude * maxVelocity)
            * teleopVelocityCoefficient;

    double yVelocity =
        (FieldConstants.isBlueAlliance() ? -yMagnitude * maxVelocity : yMagnitude * maxVelocity)
            * teleopVelocityCoefficient;

    double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(
            swerveInputs.Speeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
            swerveInputs.Pose.getRotation()),
        swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
  }

  public void resetTranslationAndRotation(Pose2d pose2d) {
    resetTranslation(pose2d);
    resetRotation(pose2d.getRotation());
  }

  public void resetRotation(Rotation2d rotation2d) {
    io.resetToParamaterizedRotation(rotation2d);
  }

  public void resetTranslation(Pose2d pose) {
    io.resetRobotTranslation(pose.getTranslation());
  }

  public void resetRotationBasedOnAlliance() {
    io.resetRotation();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public void setTargetRotation(Rotation2d desiredRotation) {
    setWantedState(WantedState.ROTATION_LOCK);
    this.desiredRotationForRotationLockState = desiredRotation;
  }

  public boolean isAtDriveToPointSetpoint() {
    var distance =
        desiredPoseForDriveToPoint
            .getTranslation()
            .minus(swerveInputs.Pose.getTranslation())
            .getNorm();
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/distanceFromEndpoint", distance);
    return MathUtil.isNear(0.0, distance, TRANSLATION_ERROR_MARGIN_METERS_DRIVE_TO_POINT);
  }

  public boolean isAtDesiredRotation() {
    return isAtDesiredRotation(Units.degreesToRadians(10.0));
  }

  public boolean isAtDesiredRotation(double tolerance) {
    return driveAtAngle.HeadingController.getPositionError() < tolerance;
  }

  public double getDistanceFromDriveToPointSetpoint() {
    var diff =
        desiredPoseForDriveToPoint
            .getTranslation()
            .minus(swerveInputs.Pose.getTranslation())
            .getNorm();
    Logger.recordOutput("DistanceFromDriveToPointSetpoint", diff);
    return diff;
  }

  public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
    this.teleopVelocityCoefficient = teleopVelocityCoefficient;
  }

  public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
    this.rotationVelocityCoefficient = rotationVelocityCoefficient;
  }

  public boolean isAtChoreoSetpoint() {
    if (systemState != SystemState.CHOREO_PATH) {
      return false;
    }
    return MathUtil.isNear(
            desiredChoreoTrajectory.getFinalPose(false).get().getX(), swerveInputs.Pose.getX(), 0.1)
        && MathUtil.isNear(
            desiredChoreoTrajectory.getFinalPose(false).get().getY(),
            swerveInputs.Pose.getY(),
            0.1);
  }

  public boolean isAtEndOfChoreoTrajectoryOrDriveToPoint() {
    if (desiredChoreoTrajectory != null) {
      return (MathUtil.isNear(
                  desiredChoreoTrajectory.getFinalPose(false).get().getX(),
                  swerveInputs.Pose.getX(),
                  0.1)
              && MathUtil.isNear(
                  desiredChoreoTrajectory.getFinalPose(false).get().getY(),
                  swerveInputs.Pose.getY(),
                  0.1))
          || isAtDriveToPointSetpoint();
    } else {
      return isAtDriveToPointSetpoint();
    }
  }

  public double getRobotDistanceFromChoreoEndpoint() {
    Logger.recordOutput("Choreo/FinalPose", desiredChoreoTrajectory.getFinalPose(false).get());
    var distance =
        Math.abs(
            desiredChoreoTrajectory
                .getFinalPose(false)
                .get()
                .minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry())
                .getTranslation()
                .getNorm());
    Logger.recordOutput("Choreo/DistanceFromEndpoint", distance);
    return distance;
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d questRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> questMeasurementStdDevs) {
    io.addQuestPose(questRobotPoseMeters, timestampSeconds, questMeasurementStdDevs);
  }
}
