package frc.robot.Config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.CanDeviceID;
import java.util.List;

public class BruinRobotConfig {
  private static final CANBus CANIVORE_CANBUS = new CANBus("CANivore");

  // Insert can ID's
  public final CanDeviceID GYRO = new CanDeviceID(-1);

  public final CanDeviceID FRONT_LEFT_DRIVE_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID FRONT_LEFT_STEER_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID FRONT_LEFT_STEER_ENCODER = new CanDeviceID(-1);

  public final CanDeviceID FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID FRONT_RIGHT_STEER_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID FRONT_RIGHT_STEER_ENCODER = new CanDeviceID(-1);

  public final CanDeviceID BACK_LEFT_DRIVE_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID BACK_LEFT_STEER_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID BACK_LEFT_STEER_ENCODER = new CanDeviceID(-1);

  public final CanDeviceID BACK_RIGHT_DRIVE_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID BACK_RIGHT_STEER_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID BACK_RIGHT_STEER_ENCODER = new CanDeviceID(-1);

  public final CanDeviceID ROTATION_MOTOR = new CanDeviceID(-1, CANIVORE_CANBUS);
  public final CanDeviceID ELEVATION_MOTOR = new CanDeviceID(-1, CANIVORE_CANBUS);
  public final CanDeviceID SHOOTER_MOTOR = new CanDeviceID(-1, CANIVORE_CANBUS);

  public final CanDeviceID ELEVATION_CANCODER = new CanDeviceID(-1, CANIVORE_CANBUS);

  public final CanDeviceID CLIMBER_LEFT_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID CLIMBER_LEFT_CANCODER = new CanDeviceID(-1);
  public final CanDeviceID CLIMBER_RIGHT_MOTOR = new CanDeviceID(-1);
  public final CanDeviceID CLIMBER_RIGHT_CANCODER = new CanDeviceID(-1);

  /**
   * Wheel radius in meters. Accuracy in these measurements affects wheel odometry which measures
   * distance as a function of the number of rotations * wheel circumference.
   */

  // TODO: find and fill
  private final double WHEEL_RADIUS_METERS = 99999999;

  /** Ratio between the drive motor shaft and the output shaft the wheel is mounted on. */

  // TODO: find and fill
  private final double DRIVE_GEAR_RATIO = 9999999999.999;

  /** Ratio between the steer motor shaft and the steer output shaft. */

  // TODO: find and fill
  private final double STEER_GEAR_RATIO = 99999999999999.9999999999;

  /**
   * The coupled gear ratio between the CanCoder and the drive motor. Every 1 rotation of the steer
   * motor results in coupled ratio of drive turns.
   */

  // TODO: find and fill
  private final double COUPLING_GEAR_RATIO = 0.0;

  /**
   * Wheelbase length is the distance between the front and back wheels. Positive x values represent
   * moving towards the front of the robot
   */

  // TODO: find and fill
  private final double WHEELBASE_LENGTH_METERS = 9999999999.99999;

  /**
   * Wheel track width is the distance between the left and right wheels. Positive y values
   * represent moving towards the left of the robot.
   */

  // TODO: find and fill
  private final double WHEEL_TRACK_WIDTH_METERS = 9999999999999999.9999;

  /** The maximum speed of the robot in meters per second. */

  // TODO: find and fill
  private final double MAX_SPEED_METERS_PER_SECOND = 999.9999;

  // CANcoder offsets of the swerve modules

  // TODO: find and fill
  private final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = 0.0;
  private final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = 0.0;
  private final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.0;
  private final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0.0;

  // TODO: find and fill
  private final int GYRO_MOUNTING_ANGLE = 0;
  private final double GYRO_ERROR = 1.6;

  // Robot configuration
  private final SwerveDrivetrainConstants swerveDrivetrainConstants;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      moduleConstants;
  private final VisionConfig photonVisionConfig;
  private final VisionConfig questNavConfig;
  private final TurretConfig turretConfig;
  private final ClimberConfig climberConfig;

  // PathPlanner config constants
  private final double ROBOT_MASS_KG = 99999999999999.99999;
  private final double ROBOT_MOI = 999999.99999999;
  private final double WHEEL_COF = 1.2;
  private RobotConfig PP_CONFIG;

  @SuppressWarnings("unchecked")
  public BruinRobotConfig() {
    Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
    pigeon2Configuration.MountPose.MountPoseYaw = GYRO_MOUNTING_ANGLE;
    pigeon2Configuration.GyroTrim.GyroScalarY = GYRO_ERROR;

    swerveDrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName("rio")
            .withPigeon2Id(GYRO.getDeviceNumber())
            .withPigeon2Configs(pigeon2Configuration);

    // TODO: set ALL of these constants
    moduleConstants = new SwerveModuleConstants[4];
    moduleConstants[0] =
        new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true)
            .withSteerMotorInverted(true)
            .withEncoderInverted(false)
            .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
            .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

    moduleConstants[1] =
        new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(FRONT_RIGHT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true)
            .withSteerMotorInverted(true)
            .withEncoderInverted(false)
            .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
            .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

    moduleConstants[2] =
        new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(BACK_LEFT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(BACK_LEFT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true)
            .withSteerMotorInverted(true)
            .withEncoderInverted(false)
            .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
            .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

    moduleConstants[3] =
        new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(BACK_RIGHT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(BACK_RIGHT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true)
            .withSteerMotorInverted(true)
            .withEncoderInverted(false)
            .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
            .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

    PP_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                WHEEL_RADIUS_METERS,
                MAX_SPEED_METERS_PER_SECOND,
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_GEAR_RATIO),
                120,
                1),
            getModuleTranslations());

    // TODO: find and fill
    photonVisionConfig =
        new VisionConfig("Photonvision Camera 1")
            .withHeightOffset(Units.inchesToMeters(0))
            .withLengthOffset(Units.inchesToMeters(0))
            .withWidthOffset(Units.inchesToMeters(0));
    questNavConfig =
        new VisionConfig("Questnav")
            .withHeightOffset(Units.inchesToMeters(0))
            .withLengthOffset(Units.inchesToMeters(0))
            .withWidthOffset(Units.inchesToMeters(0));

    // Turret Constants
    turretConfig =
        new TurretConfig()
            .withRotationKp(0.0)
            .withRotationKi(0)
            .withRotationKd(0)
            .withElevationKp(0)
            .withElevationKi(0)
            .withElevationKd(0)
            .withShooterKp(0)
            .withShooterKi(0)
            .withShooterKd(0);

    // Climber Constants
    climberConfig = new ClimberConfig().withclimberKp(0).withclimberKi(0).withclimberKd(0);
  }

  public SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
    return swerveDrivetrainConstants;
  }

  public RobotConfig getRobotConfig() {
    return PP_CONFIG;
  }

  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      getModuleConstants() {
    return moduleConstants;
  }

  public List<VisionConfig> getVisionConfigurations() {
    return List.of(photonVisionConfig, questNavConfig);
  }

  public TurretConfig getTurretConfig() {
    return turretConfig;
  }

  public ClimberConfig getClimberConfig() {
    return climberConfig;
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(moduleConstants[0].LocationX, moduleConstants[0].LocationY),
      new Translation2d(moduleConstants[1].LocationX, moduleConstants[1].LocationY),
      new Translation2d(moduleConstants[2].LocationX, moduleConstants[2].LocationY),
      new Translation2d(moduleConstants[3].LocationX, moduleConstants[3].LocationY)
    };
  }
}
