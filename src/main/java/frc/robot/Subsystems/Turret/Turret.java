package frc.robot.Subsystems.Turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants.ShooterCalculationConstants;
import frc.robot.Constant.Constants.TurretConstants;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Turret.Elevation.ElevationIO;
import frc.robot.Subsystems.Turret.Elevation.ElevationIOInputsAutoLogged;
import frc.robot.Subsystems.Turret.Rotation.RotationIO;
import frc.robot.Subsystems.Turret.Rotation.RotationIOInputsAutoLogged;
import frc.robot.Subsystems.Turret.Shooter.ShooterIO;
import frc.robot.Subsystems.Turret.Shooter.ShooterIOInputsAutoLogged;
import frc.robot.Util.SubsystemDataProcessor;
import frc.robot.Util.TurretMeasurables;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  // ─── Rotation limits ─────────────────────────────────────────────────────────
  private static final double MIN_ANGLE = Math.toRadians(-195);
  private static final double MAX_ANGLE = Math.toRadians(195);
  private static final double ANGLE_EPSILON = 1e-9;

  // ─── Setpoint tolerances ─────────────────────────────────────────────────────
  private static final double SHOOTER_SPEED_TOLERANCE_FRACTION = 0.02; // 2% of setpoint
  private static final double ANGLE_SETPOINT_TOLERANCE_RADIANS = 0.1;

  // ─── Distance thresholds for shot profile selection (meters) ─────────────────
  // Scoring profiles (all high arc, turret height = 17in / 0.4318m):
  //   0    – 2m   : dead zone — top-entry impossible, elevationAngleOutOfBounds always fires
  //   2    – 3m   : CLOSE  7.5 m/s  launch 68°–77°  descent 51°–68°  ✓
  //   3    – 4.5m : FAR    8.5 m/s  launch 65°–75°  descent 53°–69°  ✓
  //   4.5  – 6.1m : MID    10  m/s  launch 68°–75°  descent 62°–71°  ✓
  //   >6.1m       : scoring not attempted — use passing
  private static final double MIN_SCORING_DISTANCE_METERS = 2.0;
  private static final double CLOSE_RANGE_THRESHOLD_METERS = 3.0;
  private static final double FAR_RANGE_THRESHOLD_METERS = 4.5;
  private static final double MAX_SCORING_DISTANCE_METERS = 6.1;

  // Passing: reuse scoring profiles up to FAR, then step through passing tiers
  private static final double PASSING_SLOW_MAX_DISTANCE_METERS = 10.5;

  // ─── Hardware IO ─────────────────────────────────────────────────────────────
  private final ElevationIO elevationIO;
  private final RotationIO rotationIO;
  private final ShooterIO shooterIO;

  private final ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private final RotationIOInputsAutoLogged rotationInputs = new RotationIOInputsAutoLogged();
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  // ─── Turret state ─────────────────────────────────────────────────────────────
  private TurretMeasurables currentTurretMeasurables;
  private TurretMeasurables wantedTurretMeasurables;

  private Pose2d desiredPose;
  private double desiredHeight;
  private Pose2d turretPose;
  private Pose2d predictedTurretPose;
  private ChassisSpeeds chassisSpeeds;
  private boolean atGoal;

  /**
   * True for any periodic cycle in which the physics-calculated elevation angle exceeds the
   * mechanical limits and must be clamped. Cleared each periodic.
   */
  @AutoLogOutput private boolean elevationAngleOutOfBounds = false;

  // ─── Shot profile ─────────────────────────────────────────────────────────────
  /**
   * Bundles the three parameters that change with distance so they are always kept consistent and
   * pulled from Constants in one place.
   *
   * <p>ballSpeedMetersPerSec — used in the ballistic physics calculation flywheelRadiansPerSec —
   * sent to the flywheel motor (hardware units) useHighArc — selects the +√ or −√ root of the
   * trajectory formula
   */
  private static final class ShotProfile {
    final double ballSpeedMetersPerSec;
    final double flywheelRadiansPerSec;
    final boolean useHighArc;

    ShotProfile(double ballSpeedMetersPerSec, double flywheelRadiansPerSec, boolean useHighArc) {
      this.ballSpeedMetersPerSec = ballSpeedMetersPerSec;
      this.flywheelRadiansPerSec = flywheelRadiansPerSec;
      this.useHighArc = useHighArc;
    }
  }

  // ─── Enums ───────────────────────────────────────────────────────────────────
  public enum TurretSystemState {
    TRACKING_TARGET,
    IDLE,
    ACTIVE_SHOOTING,
    ACTIVE_PASSING,
    STOW,
    ZERO_HUB,
    ZERO_PASSING,
    TESTING,
    PRACTICE_SHOOTING
  }

  public enum TurretWantedState {
    IDLE,
    SHOOT_SCORE,
    PASS_TO_ALLIANCE,
    PASSIVE_TRACK,
    STOW,
    ZERO_HUB,
    ZERO_PASSING,
    TESTING,
    PRACTICE_SHOOTING
  }

  @AutoLogOutput private TurretSystemState systemState = TurretSystemState.IDLE;
  @AutoLogOutput private TurretWantedState wantedState = TurretWantedState.IDLE;

  // ─── Constructor ─────────────────────────────────────────────────────────────
  public Turret(ElevationIO elevationIO, RotationIO rotationIO, ShooterIO shooterIO) {
    this.elevationIO = elevationIO;
    this.rotationIO = rotationIO;
    this.shooterIO = shooterIO;

    currentTurretMeasurables = new TurretMeasurables(new Rotation2d(), new Rotation2d(), 0);
    wantedTurretMeasurables = new TurretMeasurables(new Rotation2d(), new Rotation2d(), 0);
    atGoal = true;

    SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
        () -> {
          synchronized (shooterInputs) {
            shooterIO.updateInputs(shooterInputs);
          }
        },
        shooterIO);
  }

  // ─── Periodic ────────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    elevationIO.updateInputs(elevationInputs);
    rotationIO.updateInputs(rotationInputs);
    synchronized (shooterInputs) {
      Logger.processInputs("Subsystems/elevation", elevationInputs);
      Logger.processInputs("Subsystems/rotation", rotationInputs);
      Logger.processInputs("Subsystems/shooter", shooterInputs);

      chassisSpeeds = Robotstate.getInstance().getRobotChassisSpeeds();

      turretPose =
          Robotstate.getInstance()
              .getRobotPoseFromSwerveDriveOdometry()
              .transformBy(TurretConstants.TURRET_TRANSFORM);

      predictedTurretPose =
          new Pose2d(
              turretPose.getX()
                  + (chassisSpeeds.vxMetersPerSecond * ShooterCalculationConstants.TIME_DELAY),
              turretPose.getY()
                  + (chassisSpeeds.vyMetersPerSecond * ShooterCalculationConstants.TIME_DELAY),
              turretPose
                  .getRotation()
                  .plus(
                      new Rotation2d(
                          chassisSpeeds.omegaRadiansPerSecond
                              * ShooterCalculationConstants.TIME_DELAY)));

      currentTurretMeasurables =
          new TurretMeasurables(
              elevationInputs.elevationAngle,
              rotationInputs.rotationAngle,
              shooterInputs.shooterVelocityRadPerSec);

      systemState = handleStateTransitions();
      elevationAngleOutOfBounds = false; // reset — convertToBoundedTurretAngle sets if needed
      applyStates();
      atGoal = atSetpoint();

      System.out.println(
          "Desired Rotation Angle: "
              + Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry());
    }
  }

  // ─── State machine ───────────────────────────────────────────────────────────
  public TurretSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return TurretSystemState.IDLE;
      case SHOOT_SCORE:
        return TurretSystemState.ACTIVE_SHOOTING;
      case PASS_TO_ALLIANCE:
        return TurretSystemState.ACTIVE_PASSING;
      case PASSIVE_TRACK:
        return TurretSystemState.TRACKING_TARGET;
      case TESTING:
        return TurretSystemState.TESTING;
      case ZERO_HUB:
        return TurretSystemState.ZERO_HUB;
      case ZERO_PASSING:
        return TurretSystemState.ZERO_PASSING;
      case STOW:
        return TurretSystemState.STOW;
      case PRACTICE_SHOOTING:
        return TurretSystemState.PRACTICE_SHOOTING;
      default:
        return TurretSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        rotationIO.setVoltage(0);
        elevationIO.setVoltage(0);
        shooterIO.setVoltage(0);
        atGoal = true;
        break;

      case TRACKING_TARGET:
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT + getScoringHeightOffset(desiredPose);
        performShootingCalc(selectShotProfile(desiredPose));
        goToWantedState();
        break;

      case ACTIVE_SHOOTING:
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT + getScoringHeightOffset(desiredPose);
        performShootingCalc(selectShotProfile(desiredPose));
        goToWantedState();
        break;

      case ACTIVE_PASSING:
        // Passing always uses high arc for hang time so alliance partners can receive.
        // selectPassingProfile picks the slowest speed that still reaches the target,
        // saving voltage compared to running at full shooting power.
        // TODO: replace getScoringPose() with a dedicated alliance passing pose.
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;
        performShootingCalc(selectPassingProfile(desiredPose));
        goToWantedState();
        break;

      case TESTING:
        wantedTurretMeasurables =
            new TurretMeasurables(Rotation2d.fromDegrees(70), Rotation2d.fromDegrees(90), 0);
        goToWantedState();
        break;

      case ZERO_HUB:
        wantedTurretMeasurables =
            new TurretMeasurables(
                Rotation2d.fromRadians(TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS),
                Rotation2d.fromDegrees(0),
                getWantedShooterSpeed(FieldConstants.getScoringPose()));
        goToWantedState();
        break;
      case ZERO_PASSING:
        wantedTurretMeasurables =
            new TurretMeasurables(
                Rotation2d.fromRadians(TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS),
                Rotation2d.fromDegrees(0),
                getWantedShooterSpeed(FieldConstants.getPassingPose()));
        break;

      case STOW:
        wantedTurretMeasurables =
            new TurretMeasurables(
                Rotation2d.fromRadians(TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS),
                Rotation2d.fromDegrees(90),
                TurretConstants.SHOOTER_CLOSE_RADIANS_PER_SEC);
        goToWantedState();
        break;

      case PRACTICE_SHOOTING:
        shooterIO.setVelo(RadiansPerSecond.of(30));
        break;
    }
  }

  // ─── Shot profile selection ───────────────────────────────────────────────────

  /**
   * Returns the appropriate ShotProfile for a scoring shot based on distance. All tiers use high
   * arc for top-entry descent angle.
   *
   * <p>< 2m : dead zone — elevationAngleOutOfBounds will always fire 2 – 3m : CLOSE 7.5 m/s launch
   * 68°–77° descent 51°–68° 3 – 4.5m: FAR 8.5 m/s launch 65°–75° descent 53°–69° 4.5–6.1m: MID 10
   * m/s launch 68°–75° descent 62°–71° > 6.1m : MID profile still returned — command layer should
   * prefer passing
   */
  private ShotProfile selectShotProfile(Pose2d target) {
    double distance = target.getTranslation().getDistance(predictedTurretPose.getTranslation());

    Logger.recordOutput("Turret/distanceToTargetMeters", distance);

    if (distance < CLOSE_RANGE_THRESHOLD_METERS) {
      Logger.recordOutput("Turret/shotProfile", "CLOSE");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_CLOSE,
          TurretConstants.SHOOTER_CLOSE_RADIANS_PER_SEC,
          true);
    } else if (distance > FAR_RANGE_THRESHOLD_METERS) {
      Logger.recordOutput("Turret/shotProfile", "FAR");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_FAR,
          TurretConstants.SHOOTER_FAR_RADIANS_PER_SEC,
          true);
    } else {
      Logger.recordOutput("Turret/shotProfile", "MID");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_MID,
          TurretConstants.SHOOTER_MID_RADIANS_PER_SEC,
          true);
    }
  }

  private double getWantedShooterSpeed(Pose2d target) {
    double distance = target.getTranslation().getDistance(predictedTurretPose.getTranslation());

    if (distance < CLOSE_RANGE_THRESHOLD_METERS) {
      return TurretConstants.SHOOTER_CLOSE_RADIANS_PER_SEC;
    } else if (distance > FAR_RANGE_THRESHOLD_METERS) {
      return TurretConstants.SHOOTER_FAR_RADIANS_PER_SEC;
    } else {
      return TurretConstants.SHOOTER_MID_RADIANS_PER_SEC;
    }
  }

  /**
   * Selects the minimum-voltage profile for a passing shot. Reuses scoring profiles at close range
   * since they are already the cheapest option at those distances.
   *
   * <p>0 – 3m : CLOSE 7.5 m/s 3 – 4.5m : FAR 8.5 m/s 4.5– 10.5m: PASSING_SLOW 11 m/s 10.5m+ :
   * PASSING_FAST 12 m/s
   */
  private ShotProfile selectPassingProfile(Pose2d target) {
    double distance = target.getTranslation().getDistance(predictedTurretPose.getTranslation());

    Logger.recordOutput("Turret/distanceToTargetMeters", distance);

    if (distance < CLOSE_RANGE_THRESHOLD_METERS) {
      Logger.recordOutput("Turret/shotProfile", "PASSING_REUSE_CLOSE");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_CLOSE,
          TurretConstants.SHOOTER_CLOSE_RADIANS_PER_SEC,
          true);
    } else if (distance < FAR_RANGE_THRESHOLD_METERS) {
      Logger.recordOutput("Turret/shotProfile", "PASSING_REUSE_FAR");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_FAR,
          TurretConstants.SHOOTER_FAR_RADIANS_PER_SEC,
          true);
    } else if (distance <= PASSING_SLOW_MAX_DISTANCE_METERS) {
      Logger.recordOutput("Turret/shotProfile", "PASSING_SLOW_HIGH_ARC");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_PASSING_SLOW,
          TurretConstants.SHOOTER_PASSING_SLOW_RADIANS_PER_SEC,
          true);
    } else {
      Logger.recordOutput("Turret/shotProfile", "PASSING_FAST_HIGH_ARC");
      return new ShotProfile(
          ShooterCalculationConstants.GEOMETRY_VELOCITY_ULTRA_FAR,
          TurretConstants.SHOOTER_ULTRA_FAR_RADIANS_PER_SEC,
          true);
    }
  }

  /**
   * Computes wantedTurretMeasurables (elevation, rotation, flywheel speed) to hit desiredPose at
   * desiredHeight while compensating for robot inertia.
   *
   * <p>Physics note: V_shooter_robot = V_noInertia_field − V_robot_field At launch: V_ball_field =
   * V_shooter_robot + V_robot_field = V_noInertia_field ✓
   *
   * <p>Unit note: updateWithCartesianVector() returns a speed magnitude in m/s (ball speed). After
   * the vector math, wantedTurretMeasurables.shooterRadiansPerSec is overwritten with the profile's
   * flywheel rad/s so that hardware units and physics units are never mixed.
   */
  private void performShootingCalc(ShotProfile profile) {
    Rotation2d fieldAngle = findFieldCentricAngleToTarget(desiredPose);
    Rotation2d elevAngle =
        calculateElevationAngle(desiredPose, profile.ballSpeedMetersPerSec, profile.useHighArc);

    Logger.recordOutput("Turret/fieldCentricAngleDeg", fieldAngle.getDegrees());
    Logger.recordOutput("Turret/elevationAngleDeg", elevAngle.getDegrees());
    Logger.recordOutput("Turret/ballSpeedMps", profile.ballSpeedMetersPerSec);
    Logger.recordOutput("Turret/useHighArc", profile.useHighArc);

    TurretMeasurables noInertiaMeasurables =
        new TurretMeasurables(elevAngle, fieldAngle, profile.ballSpeedMetersPerSec);

    Vector<N3> robotSpeedVector =
        VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);

    wantedTurretMeasurables.updateWithCartesianVector(
        noInertiaMeasurables.getVector().minus(robotSpeedVector));

    // Replace the m/s magnitude produced by updateWithCartesianVector with the
    // correct flywheel setpoint in rad/s for this distance range.
    wantedTurretMeasurables.shooterRadiansPerSec = profile.flywheelRadiansPerSec;

    convertToRobotRelativeNonBounded();
    convertToBoundedTurretAngle();
  }

  // ─── Output helpers ──────────────────────────────────────────────────────────

  public void setFieldRelativeTarget(TurretMeasurables wantedMeasurables) {
    this.wantedTurretMeasurables = wantedMeasurables;
  }

  private void goToWantedState() {
    rotationIO.setRotationAngle(wantedTurretMeasurables.rotationAngle.getRadians());
    elevationIO.setElevationAngle(wantedTurretMeasurables.elevationAngle);
    shooterIO.setVelo(
        AngularVelocity.ofBaseUnits(
            wantedTurretMeasurables.shooterRadiansPerSec, RadiansPerSecond));
  }

  private boolean atSetpoint() {
    double shooterTolerance =
        Math.max(1.0, Math.abs(wantedTurretMeasurables.shooterRadiansPerSec))
            * SHOOTER_SPEED_TOLERANCE_FRACTION;

    Logger.recordOutput(
        "Turret/Setpoint/currentElevationDeg",
        currentTurretMeasurables.elevationAngle.getDegrees());
    Logger.recordOutput(
        "Turret/Setpoint/wantedElevationDeg", wantedTurretMeasurables.elevationAngle.getDegrees());
    Logger.recordOutput(
        "Turret/Setpoint/currentRotationDeg", currentTurretMeasurables.rotationAngle.getDegrees());
    Logger.recordOutput(
        "Turret/Setpoint/wantedRotationDeg", wantedTurretMeasurables.rotationAngle.getDegrees());
    Logger.recordOutput(
        "Turret/Setpoint/currentShooterRadPs", currentTurretMeasurables.shooterRadiansPerSec);
    Logger.recordOutput(
        "Turret/Setpoint/wantedShooterRadPs", wantedTurretMeasurables.shooterRadiansPerSec);
    Logger.recordOutput("Turret/Setpoint/shooterTolerance", shooterTolerance);

    boolean elevationAtSetpoint =
        MathUtil.isNear(
            currentTurretMeasurables.elevationAngle.getRadians(),
            wantedTurretMeasurables.elevationAngle.getRadians(),
            ANGLE_SETPOINT_TOLERANCE_RADIANS);
    boolean rotationAtSetpoint =
        MathUtil.isNear(
            currentTurretMeasurables.rotationAngle.getRadians(),
            wantedTurretMeasurables.rotationAngle.getRadians(),
            ANGLE_SETPOINT_TOLERANCE_RADIANS);
    boolean shooterAtSetpoint =
        MathUtil.isNear(
            currentTurretMeasurables.shooterRadiansPerSec,
            wantedTurretMeasurables.shooterRadiansPerSec,
            shooterTolerance);

    Logger.recordOutput("Turret/Setpoint/elevationAtSetpoint", elevationAtSetpoint);
    Logger.recordOutput("Turret/Setpoint/rotationAtSetpoint", rotationAtSetpoint);
    Logger.recordOutput("Turret/Setpoint/shooterAtSetpoint", shooterAtSetpoint);
    Logger.recordOutput(
        "Turret/Setpoint/atSetpoint",
        elevationAtSetpoint && rotationAtSetpoint && shooterAtSetpoint);

    return elevationAtSetpoint && rotationAtSetpoint && shooterAtSetpoint;
  }

  /**
   * Returns how far above the physical rim to aim, based on distance. Aiming above the rim ensures
   * the ball is already descending when it reaches the rim level, improving top-entry angle
   * consistency. At longer range where descent angle is shallower we aim higher to compensate — but
   * speed limits mean this has diminishing returns beyond 4m.
   */
  private double getScoringHeightOffset(Pose2d target) {
    double distance = target.getTranslation().getDistance(predictedTurretPose.getTranslation());
    if (distance < CLOSE_RANGE_THRESHOLD_METERS) return 0.30; // 2–3m
    if (distance < FAR_RANGE_THRESHOLD_METERS) return 0.35; // 3–4.5m
    return 0.40; // 4.5m+
  }

  // ─── Geometry ────────────────────────────────────────────────────────────────

  public Rotation2d findFieldCentricAngleToTarget(Pose2d target) {
    return target.getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
  }

  /**
   * Solves the ballistic trajectory for the required elevation angle.
   *
   * <p>Formula derivation (range equation solved for θ): tanθ = (v² ± √(v⁴ − g(gx² + 2yv²))) / (gx)
   * +√ → high arc (more hang time, steeper angle) −√ → low arc (flatter, faster to target)
   *
   * @param target field-frame target pose
   * @param v ball exit speed in m/s
   * @param useHighArc selects the +√ (high) or −√ (low) solution
   */
  public Rotation2d calculateElevationAngle(Pose2d target, double v, boolean useHighArc) {
    double x = target.getTranslation().getDistance(predictedTurretPose.getTranslation());
    double y = desiredHeight - ShooterCalculationConstants.TURRET_HEIGHT;
    double g = ShooterCalculationConstants.GRAVITATION_CONSTANT;

    if (x < 1e-6) {
      return Rotation2d.fromRadians(Math.PI / 2.0); // directly below — shoot straight up
    }

    double v2 = v * v;
    double discriminant = v2 * v2 - g * (g * x * x + 2 * y * v2);

    if (discriminant < 0) {
      // Ball cannot reach target at this speed — clamp to steepest allowed angle
      return Rotation2d.fromRadians(TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
    }

    double sqrt = Math.sqrt(discriminant);
    double tanTheta = useHighArc ? (v2 + sqrt) / (g * x) : (v2 - sqrt) / (g * x);

    return new Rotation2d(Math.atan(tanTheta));
  }

  // ─── Coordinate conversion ────────────────────────────────────────────────────

  /**
   * Converts the wanted rotation from field-centric CCW+ to robot-relative CW+.
   *
   * <p>Step 1: θ_rel_ccw = θ_field − θ_predictedRobotHeading (still CCW+) Step 2: θ_turret =
   * −θ_rel_ccw (flip to CW+)
   *
   * <p>Uses predictedTurretPose heading — not current odometry — because the field-centric angle
   * was computed against that future heading.
   */
  private void convertToRobotRelativeNonBounded() {
    Rotation2d robotRelativeCCW =
        wantedTurretMeasurables.rotationAngle.minus(predictedTurretPose.getRotation());
    wantedTurretMeasurables.rotationAngle = robotRelativeCCW.unaryMinus();
  }

  /**
   * Maps the unbounded robot-relative angle onto [MIN_ANGLE, MAX_ANGLE] by choosing the shortest
   * path from the current unwrapped encoder position, then clamps elevation within its mechanical
   * limits.
   */
  public void convertToBoundedTurretAngle() {
    double currentTotalRadians = rotationInputs.totalRotationsUnwrapped * 2 * Math.PI;

    double closestOffset =
        wantedTurretMeasurables.rotationAngle.getRadians()
            - rotationInputs.rotationAngle.getRadians();

    if (closestOffset > Math.PI) closestOffset -= 2 * Math.PI;
    else if (closestOffset < -Math.PI) closestOffset += 2 * Math.PI;

    double finalOffset = currentTotalRadians + closestOffset;

    // Equidistant tiebreaker — choose the path closer to zero (epsilon, not ==)
    double positivePath = currentTotalRadians + Math.abs(closestOffset);
    double negativePath = currentTotalRadians - Math.abs(closestOffset);
    if (Math.abs(
            Math.IEEEremainder(positivePath, 2 * Math.PI)
                - Math.IEEEremainder(negativePath, 2 * Math.PI))
        < ANGLE_EPSILON) {
      finalOffset =
          (Math.abs(positivePath) <= Math.abs(negativePath)) ? positivePath : negativePath;
    }

    // While-loop ensures the result is always within bounds regardless of magnitude
    while (finalOffset > MAX_ANGLE) finalOffset -= 2 * Math.PI;
    while (finalOffset < MIN_ANGLE) finalOffset += 2 * Math.PI;

    if (Rotation2d.fromRadians(finalOffset).getRadians() > 0) {
      wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(finalOffset - Math.PI);
    } else {
      wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(finalOffset + Math.PI);
    }

    // wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(finalOffset);

    double rawElevation = wantedTurretMeasurables.elevationAngle.getRadians();
    double clampedElevation =
        MathUtil.clamp(
            rawElevation,
            TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS,
            TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);

    elevationAngleOutOfBounds = (rawElevation != clampedElevation);
    Logger.recordOutput("Turret/elevationAngleOutOfBounds", elevationAngleOutOfBounds);

    wantedTurretMeasurables.elevationAngle = Rotation2d.fromRadians(clampedElevation);
  }

  // ─── Public setters / getters ─────────────────────────────────────────────────

  public void setWantedState(TurretWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean getAtGoal() {
    return atGoal;
  }

  public boolean getElevationAngleOutOfBounds() {
    return elevationAngleOutOfBounds;
  }

  public void setWantedTurretMeasurables(TurretMeasurables wanted) {
    this.wantedTurretMeasurables = wanted;
  }

  public void setEncoderPositionAtBottom() {
    elevationIO.setEncoderPositionAtBottom();
  }
}
