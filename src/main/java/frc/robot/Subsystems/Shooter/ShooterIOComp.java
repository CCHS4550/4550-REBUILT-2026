package frc.robot.Subsystems.Shooter;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;
import frc.robot.Globals;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.tools.TunableNumber;

class ShooterIOComp implements ShooterIO {
        private final TalonFX flywheelMaster;
        private final TalonFX flywheelFollower;
        private final TalonFX turretMotor;
        private final TalonFX hoodMotor ;

        private final VelocityVoltage flywheelControl;

       private final CANcoder rotationEncoder;

        private LinearFilter filterTurret = LinearFilter.movingAverage(10);

        // private TunableNumber turretP = new TunableNumber("Turret Position kP",
        // Constants.PIDConstants.Turret.kP0);
        // private TunableNumber turretI = new TunableNumber("Turret Position kI",
        // Constants.PIDConstants.Turret.kI0);
        // private TunableNumber turretD = new TunableNumber("Turret Position kD",
        // Constants.PIDConstants.Turret.kD0);
        // private TunableNumber turretS = new TunableNumber("Turret Position kS",
        // Constants.PIDConstants.Turret.kS0);
        // private TunableNumber turretV = new TunableNumber("Turret Position kV",
        // Constants.PIDConstants.Turret.kV0);
        // private TunableNumber turretVelocity = new TunableNumber("Turret Position
        // Velocity", 3.0);
        // private TunableNumber turretAcceleration = new TunableNumber("Turret Position
        // Acceleration", 10.0);

        private double turretVelocity = 3.0;
        private double turretAcceleration = 10.0;
        // private TunableNumber hoodP = new TunableNumber("Hood Position kP",
        // Constants.PIDConstants.Hood.kP0);
        // private TunableNumber hoodI = new TunableNumber("Hood Position kI",
        // Constants.PIDConstants.Hood.kI0);
        // private TunableNumber hoodD = new TunableNumber("Hood Position kD",
        // Constants.PIDConstants.Hood.kD0);
        // private TunableNumber hoodS = new TunableNumber("Hood Position kS",
        // Constants.PIDConstants.Hood.kS0);
        // private TunableNumber hoodG = new TunableNumber("Hood Position kG",
        // Constants.PIDConstants.Hood.kG0);
        // private TunableNumber hoodCruiseVelocity = new TunableNumber("Hood Position
        // Velocity", 2.0);
        // private TunableNumber hoodAcceleration = new TunableNumber("Hood Position
        // Acceleration", 2.0);

        private double hoodCruiseVelocity = 2.0;
        private double hoodAcceleration = 2.0;
        private final double hoodProfileScalarFactor = 1.0;

        private final DynamicMotionMagicVoltage hoodMotionProfileRequest = new DynamicMotionMagicVoltage(0,
                        hoodCruiseVelocity,
                        hoodAcceleration);

        // private TunableNumber flywheelP = new TunableNumber("Flywheel Position kP",
        // Constants.PIDConstants.Flywheel.kP0);
        // private TunableNumber flywheelI = new TunableNumber("Flywheel Position kI",
        // Constants.PIDConstants.Flywheel.kI0);
        // private TunableNumber flywheelD = new TunableNumber("Flywheel Position kD",
        // Constants.PIDConstants.Flywheel.kD0);
        // private TunableNumber flywheelS = new TunableNumber("Flywheel Position kS",
        // Constants.PIDConstants.Flywheel.kS0);
        // private TunableNumber flywheelV = new TunableNumber("Flywheel Position kV",
        // Constants.PIDConstants.Flywheel.kV0);
        // private TunableNumber flywheelVelocity = new TunableNumber("Flywheel Position
        // Velocity", 2.0);
        // private TunableNumber flywheelAcceleration = new TunableNumber("Flywheel
        // Position Acceleration", 2.0);

        public ShooterIOComp(BruinRobotConfig config) {

                turretMotor = new TalonFX(config.ROTATION_MOTOR.getDeviceNumber(), config.ROTATION_MOTOR.getBus());
                hoodMotor = new TalonFX(config.ELEVATION_MOTOR.getDeviceNumber(), config.ELEVATION_MOTOR.getBus());
                flywheelMaster = new TalonFX(config.SHOOTER_MOTOR.getDeviceNumber(), config.SHOOTER_MOTOR.getBus());
                flywheelFollower = new TalonFX(config.SHOOTER_MOTOR_2.getDeviceNumber(), config.SHOOTER_MOTOR_2.getBus());
                rotationEncoder = new CANcoder(config.ROTATION_CANCODER.getDeviceNumber(), config.ROTATION_CANCODER.getBus());
                flywheelControl = new VelocityVoltage(0.0);


                initializingTurret = true;
                initLoops = 0;
                // Hood Motor Configuration //TODO: Gotta tune all of the configs
                // System.out.println("slope: " + SLOPE);
                TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
                hoodConfig.Slot0.kP = config.getTurretConfig().elevationKp;
                hoodConfig.Slot0.kI = config.getTurretConfig().elevationKi;
                hoodConfig.Slot0.kD = config.getTurretConfig().elevationKd;
                hoodConfig.Slot0.kS = config.getTurretConfig().elevationKs;
                hoodConfig.Slot0.kG = 0.4;
                hoodConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                hoodConfig.MotionMagic.MotionMagicAcceleration = 64.4;
                hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 75.3;

                hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
           
                hoodConfig.CurrentLimits.StatorCurrentLimit = 30;
                hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
                hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                
                Phoenix6Util.applyAndCheckConfiguration(hoodMotor, hoodConfig, 5);

                // Flywheel Configuration
                TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
                flywheelConfig.Slot0.kP = config.getTurretConfig().shooterKp;
                flywheelConfig.Slot0.kI = config.getTurretConfig().shooterKi;
                flywheelConfig.Slot0.kD = config.getTurretConfig().shooterKd;
                flywheelConfig.Slot0.kS = config.getTurretConfig().shooterKs;
                flywheelConfig.Slot0.kV = config.getTurretConfig().shooterKv;
                flywheelConfig.Feedback.SensorToMechanismRatio = 1.0;
                flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
                flywheelConfig.CurrentLimits.StatorCurrentLimit = 40;
                flywheelConfig.CurrentLimits.SupplyCurrentLimit = 90;
                flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                flywheelConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
                flywheelConfig.MotionMagic.MotionMagicAcceleration = 50;
                flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

                flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                
                Phoenix6Util.applyAndCheckConfiguration(flywheelFollower, flywheelConfig, 5);
                Phoenix6Util.applyAndCheckConfiguration(flywheelFollower, flywheelConfig, 5);
                // Turret Motor Configuration
                TalonFXConfiguration turretConfig = new TalonFXConfiguration();
                turretConfig.Slot0.kP = config.getTurretConfig().rotationKp;
                turretConfig.Slot0.kI = config.getTurretConfig().rotationKi;
                turretConfig.Slot0.kD = config.getTurretConfig().rotationKd;
                turretConfig.Slot0.kS = config.getTurretConfig().rotationKs;
                turretConfig.Slot0.kV = config.getTurretConfig().rotationKv;
                turretConfig.MotionMagic.MotionMagicAcceleration = 15;
                turretConfig.MotionMagic.MotionMagicCruiseVelocity = 13;
                turretConfig.Feedback.SensorToMechanismRatio = Constants.TurretConstants.ROTATION_GEAR_RATIO_TO_ENCODER;
                turretConfig.Feedback.RotorToSensorRatio = Constants.TurretConstants.ROTATION_GEAR_RATIO_TO_ENCODER / Constants.TurretConstants.ROTATION_GEAR_RATIO;
                turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                turretConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
                turretConfig.CurrentLimits.StatorCurrentLimit = 90.0;
                turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                turretMotor.getConfigurator().apply(turretConfig);
                turretMotor.setNeutralMode(NeutralModeValue.Brake);

                // CANcoder Configuration
                CANcoderConfiguration encoderOneConfig = new CANcoderConfiguration();
                encoderOneConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
                encoderOneConfig.MagnetSensor.MagnetOffset = -0.75; // TODO: Try calculating offset from
                                                                              // previous zero
                                                                              // data
                encoderOneConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
                rotationEncoder.getConfigurator().apply(encoderOneConfig);
              
      

                turretMotor.setPosition(Units.radiansToRotations(getRelativeTurretAngleRadians()));
        }

        @Override
        public Rotation2d getHoodAngle() {
                return motorAngleToHoodAngle(
                                                Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble())); // TODO:
                                                                                                                       // try
                                                                                                                       // getLatencyCompensatedValueAsDouble()
        }

        @Override
        public Rotation2d getTurretAngleSetpointForTrajectory(
            Translation3d _trajectorySetpoint) {
            return new Rotation2d(Math.atan2(_trajectorySetpoint.getY(),
                                                _trajectorySetpoint.getX()));
                        }

                        @Override

                        public Rotation2d getFutureSetpointEstimate(Rotation2d currentSetpoint,
                                        double driveAngularVelocity, double foresightTime) {
                                Logger.recordOutput("Shooter/Turret Drive Angular Velocity", driveAngularVelocity);
                                double predictedAngle = currentSetpoint.getRadians()
                                                - driveAngularVelocity * foresightTime;
                                return new Rotation2d(predictedAngle);
                        }

        @Override
        public void zeroTurretToEncoder() {
                turretMotor.setPosition(Units.radiansToRotations(getRelativeTurretAngleRadians()));
        }
        @Override
        public  double getFlywheelRPMSetpointForTrajectory(Translation3d _trajectorySetpoint) {
                                double v = _trajectorySetpoint.getNorm();
                                double rpm = shooterMPSToRPM(v);
                                return rpm;
                        }
        @Override
                    public  Rotation2d getHoodAngleSetpointForTrajectory(Translation3d trajectory) {
                                double dz = trajectory.getZ();
                                double dr = Math.hypot(trajectory.getX(), trajectory.getY());
                                double angleRadians = Math.atan(dz / dr);
                                return launchAngleToHoodAngle(new Rotation2d(angleRadians),
                                                shooterMPSToRPM(trajectory.getNorm()));
                        }

        @Override
        public final Rotation2d hoodAngleToMotorAngle(Rotation2d hoodAngle) {
            return Rotation2d.fromRadians(Math.PI - Units.degreesToRadians(77.312)).minus(hoodAngle);
        }
        @Override
        public final Rotation2d motorAngleToHoodAngle(Rotation2d motorAngle) {
             return Rotation2d.fromRadians(Math.PI - Units.degreesToRadians(77.312)).minus(motorAngle);
        }
        @Override
         public Rotation2d launchAngleToHoodAngle(Rotation2d launchAngle, double rpm) {
                return Rotation2d.fromDegrees(0)
                                .plus(launchAngle);
                // return Rotation2d.fromDegrees(launchAngle.getDegrees() +
                // launchAngleOffset.get());
        }
        @Override
        public double shooterMPSToRPM(double mps) {

                return (151.0 * mps - 183.0) * 0.70;
                // return 3621.1-11.9904*Math.sqrt(76609-8340*mps);
        }

        @Override
        public Rotation2d getTurretAngle() {
                return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
        }

        @Override
        public double getFlywheelRPM() {
                return flywheelMaster.getVelocity().getValueAsDouble() * 60;
        }

        @Override
        public void moveHoodToAngle(Rotation2d angle) {
                Logger.recordOutput("Shooter/Hood target angle", angle.getDegrees());
                if (angle.getRadians() > Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS) {
                        angle = Rotation2d.fromRadians(Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
                }
                if (angle.getRadians() < Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS) {
                        angle = Rotation2d.fromRadians(Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
                }
                double wantedAngle = hoodAngleToMotorAngle(angle).getRotations();
                hoodMotor.setControl(this.hoodMotionProfileRequest
                                .withPosition(
                                                wantedAngle)
                                .withVelocity(hoodCruiseVelocity * hoodProfileScalarFactor)
                                .withAcceleration(hoodAcceleration * hoodProfileScalarFactor)
                                .withSlot(0));
        }

        @Override
        public void setTurretAngle(double angle) {
                Logger.recordOutput("Shooter/Goal turret degrees", Math.toDegrees(angle));
                turretMotor.setControl(
                                new DynamicMotionMagicVoltage(Units.radiansToRotations(angle), turretVelocity,
                                                turretAcceleration));
                Logger.recordOutput("Shooter/goal motor turret degrees Er",
                                Units.rotationsToDegrees(turretMotor.getClosedLoopError().getValueAsDouble()));

        }

        @Override
        public void setFlywheelRPM(double rpm) {
                double velocitySetpoint = rpm / 60;
                flywheelMaster.setControl(
                                flywheelControl.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true));
                flywheelFollower.setControl(
                                flywheelControl.withVelocity(-velocitySetpoint).withSlot(0).withEnableFOC(true));
        }

        @Override
        public double getRelativeTurretAngleRadians() {
                double aMeas = rotationEncoder.getAbsolutePosition().getValueAsDouble();
                double bMeas = hoodMotor.getPosition().getValueAsDouble();

                double k1 = 1.0;

                double k2 = 1/7.41;

                double minTheta = Units.radiansToRotations(
                                Constants.TurretConstants.ROTATION_MINANGLE.getRadians());
                double maxTheta = Units.radiansToRotations(
                                Constants.TurretConstants.ROTATION_MAXANGLE.getRadians());

                double bestTheta = 0.0;
                double bestError = Double.POSITIVE_INFINITY;

                // Compute reasonable bounds on n
                int nMin = (int) Math.floor(k1 * minTheta + aMeas - 1);
                int nMax = (int) Math.ceil(k1 * maxTheta + aMeas);

                for (int n = nMin; n <= nMax; n++) {
                        double theta = (aMeas + n) / k1;

                        if (theta < minTheta || theta > maxTheta) {
                                continue;
                        }

                        double bPred = wrap(k2 * theta);
                        double err = wrapDiff(bMeas, bPred);

                        double error = err * err;

                        if (error < bestError) {
                                bestError = error;
                                bestTheta = theta;
                        }
                }

                return Units.rotationsToRadians(bestTheta);
        }

        private double wrap(double x) {
                return x - Math.floor(x);
        }

        private double wrapDiff(double a, double b) {
                double d = a - b;
                if (d > 0.5)
                        d -= 1.0;
                if (d < -0.5)
                        d += 1.0;
                return d;
        }

        @Override
        public void setHoodAngle(Rotation2d angle) { // DON'T USE
                // hoodMotor.setPosition(angle.getRotations());
        }

        @Override
        public void setFlywheelPercent(double percent) {
                flywheelMaster.set(percent);
                flywheelFollower.set(-percent);
        }

        @Override
        public double getFlywheelCurrent() {
                return flywheelMaster.getStatorCurrent().getValueAsDouble();
        }

        @Override
        public double getFlywheelAcceleration() {
                return flywheelMaster.getAcceleration().getValueAsDouble();
        }

        @Override
        public double getHoodCurrent() {
                return hoodMotor.getStatorCurrent().getValueAsDouble();
        }

        @Override
        public double getTurretCurrent() {
                return turretMotor.getStatorCurrent().getValueAsDouble();
        }

        private boolean initializingTurret;
        private int initLoops;
        private ArrayList<Double> firstTurretAngles = new ArrayList<>();
        private int numberSkips;

        @Override
        public void updateInputs() {
                Globals.turretAngle = getTurretAngle();
                Logger.recordOutput("Shooter/Turret vel unfiltered",
                                Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()));
                filterTurret.calculate(Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()));
                Globals.turretVelocity = filterTurret.lastValue();
                if (initializingTurret) {
                        initLoops++;
                        firstTurretAngles.add(getRelativeTurretAngleRadians());
                        if (firstTurretAngles.size() > 1) {
                                double tempError = firstTurretAngles.get(firstTurretAngles.size() - 2)
                                                - firstTurretAngles.get(firstTurretAngles.size() - 1);
                                if (tempError > Math.toRadians(10.0)) {
                                        numberSkips++;
                                }
                        }
                }

                if (initializingTurret && initLoops > 10) {
                        if (numberSkips < 4) {
                                initializingTurret = false;
                                initLoops = 0;
                                firstTurretAngles.sort(null);
                                double median = firstTurretAngles.get(firstTurretAngles.size() / 2);
                                turretMotor.setPosition(Units.radiansToRotations(median));
                                System.out.println(
                                                "Motor Zeroed succesfully at " + Math.toDegrees(median) + " degrees");
                                System.out.println("List: " + firstTurretAngles.toString());
                        } else {
                                initLoops = 0;
                                numberSkips = 0;
                                firstTurretAngles.clear();
                        }
                }

                Logger.recordOutput("Testing/initializingTurret", initializingTurret);
                Logger.recordOutput("Testing/initLoops", initLoops);
                Logger.recordOutput("Testing/firstTurretAngles", firstTurretAngles.toString());
                Logger.recordOutput("Testing/numberSkips", numberSkips);
                Logger.recordOutput("Shooter/Relative Turret Angle", Math.toDegrees(getRelativeTurretAngleRadians()));
                Logger.recordOutput("Shooter/Motor Turret Angle",
                                Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble()));
                Logger.recordOutput("Shooter/Turret Error Degrees",
                                turretMotor.getClosedLoopError().getValueAsDouble() * 360.0);
                // if (turretP.changed() || turretI.changed() || turretD.changed() ||
                // turretS.changed() || turretV.changed()) {
                // System.out.println("Updating Turret PID Constants");
                // TalonFXConfiguration turretConfig = new TalonFXConfiguration();
                // turretConfig.Slot0.kP = turretP.get();
                // turretConfig.Slot0.kI = turretI.get();
                // turretConfig.Slot0.kD = turretD.get();
                // turretConfig.Slot0.kS = turretS.get();
                // turretConfig.Slot0.kV = turretV.get();
                // turretConfig.MotionMagic.MotionMagicAcceleration = turretAcceleration;
                // turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretVelocity;
                // turretConfig.Feedback.SensorToMechanismRatio =
                // Constants.Ratios.Shooter.TURRET_GEAR_RATIO;
                // turretConfig.Feedback.RotorToSensorRatio = 1.0;
                // turretConfig.CurrentLimits.StatorCurrentLimit = 67;
                // turretConfig.CurrentLimits.SupplyCurrentLimit = 67;
                // turretConfig.Feedback.FeedbackSensorSource =
                // FeedbackSensorSourceValue.RotorSensor;
                // turretMotor.getConfigurator().apply(turretConfig);
                // }
                // if (hoodP.changed() || hoodI.changed() || hoodD.changed() || hoodS.changed()
                // || hoodG.changed()) {
                // System.out.println("Updating Hood PID Constants");
                // TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
                // hoodConfig.Slot0.kP = hoodP.get();
                // hoodConfig.Slot0.kI = hoodI.get();
                // hoodConfig.Slot0.kD = hoodD.get();
                // hoodConfig.Slot0.kS = hoodS.get();
                // hoodConfig.Slot0.kG = hoodG.get();
                // hoodConfig.Feedback.SensorToMechanismRatio =
                // Constants.Ratios.Shooter.HOOD_ENCODER_TO_MECHANISM_GEAR_RATIO;
                // hoodConfig.Feedback.RotorToSensorRatio =
                // Constants.Ratios.Shooter.HOOD_MOTOR_TO_ENCODER_GEAR_RATIO;
                // hoodConfig.CurrentLimits.StatorCurrentLimit = 67;
                // hoodConfig.CurrentLimits.SupplyCurrentLimit = 67;
                // hoodConfig.Feedback.FeedbackSensorSource =
                // FeedbackSensorSourceValue.RemoteCANcoder;
                // hoodConfig.Feedback.FeedbackRemoteSensorID =
                // Constants.CANInfo.HOOD_CANCODER_ID;
                // hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                // hoodMotor.getConfigurator().apply(hoodConfig);
                // }
                // if (flywheelP.changed() || flywheelI.changed() || flywheelD.changed() ||
                // flywheelS.changed() || flywheelV
                // .changed()) {
                // System.out.println("Updating Flywheel PID Constants");
                // TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
                // flywheelConfig.Slot0.kP = flywheelP.get();
                // flywheelConfig.Slot0.kI = flywheelI.get();
                // flywheelConfig.Slot0.kD = flywheelD.get();
                // flywheelConfig.Slot0.kS = flywheelS.get();
                // flywheelConfig.Slot0.kV = flywheelV.get();
                // flywheelConfig.Feedback.SensorToMechanismRatio =
                // Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                // flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
                // flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
                // flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80;
                // flywheelConfig.Feedback.FeedbackSensorSource =
                // FeedbackSensorSourceValue.RotorSensor;
                // flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                // flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                // flywheelMaster.getConfigurator().apply(flywheelConfig);
                // flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
                // flywheelFollower.getConfigurator().apply(flywheelConfig);
                // flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
                // }
        }
}