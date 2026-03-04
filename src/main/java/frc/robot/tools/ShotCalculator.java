package frc.robot.tools;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constant.Constants;

public class ShotCalculator {
        public static class ShotSolution {
                public final Rotation2d hoodAngle;
                public final double flywheelRPM;
                public final Rotation2d turretAngle;
                public final double distanceToTarget;
                public final double timeOfFlight;
                public Optional<ChassisSpeeds> robotVelocity = Optional.empty();

                public ShotSolution() {
                        this.hoodAngle = new Rotation2d();
                        this.flywheelRPM = 0.0;
                        this.turretAngle = new Rotation2d();
                        this.distanceToTarget = 0.0;
                        this.timeOfFlight = 0.0;
                }

                public void setRobotVelocity(ChassisSpeeds robotVelocity) {
                        this.robotVelocity = Optional.of(robotVelocity);
                }

                public ShotSolution(Rotation2d hoodAngle, double flywheelRPM, Rotation2d turretAngle,
                                double distanceToTarget, double timeOfFlight) {
                        this.hoodAngle = hoodAngle;
                        this.flywheelRPM = flywheelRPM;
                        this.turretAngle = turretAngle;
                        this.distanceToTarget = distanceToTarget;
                        this.timeOfFlight = timeOfFlight;
                }

                public ShotSolution rotateTurretAngle(Rotation2d rotation) {
                        ShotSolution rotated = new ShotSolution(
                                        this.hoodAngle,
                                        this.flywheelRPM,
                                        this.turretAngle.plus(rotation),
                                        this.distanceToTarget,
                                        this.timeOfFlight);
                        rotated.robotVelocity = this.robotVelocity;
                        return rotated;
                }

                public ShotSolution rotateHoodAngle(Rotation2d rotation) {
                        ShotSolution rotated = new ShotSolution(
                                        this.hoodAngle.plus(rotation),
                                        this.flywheelRPM,
                                        this.turretAngle,
                                        this.distanceToTarget,
                                        this.timeOfFlight);
                        rotated.robotVelocity = this.robotVelocity;
                        return rotated;
                }

                public ShotSolution addRPM(double rpm) {
                        ShotSolution adjusted = new ShotSolution(
                                        this.hoodAngle,
                                        this.flywheelRPM + rpm,
                                        this.turretAngle,
                                        this.distanceToTarget,
                                        this.timeOfFlight);
                        adjusted.robotVelocity = this.robotVelocity;
                        return adjusted;
                }
        }

        // Input: distance to target in meters, Output: hood angle (Rotation2d)
        private final static InterpolatingTreeMap<Double, Rotation2d> hubHoodAngleMap = new InterpolatingTreeMap<>(
                        InverseInterpolator.forDouble(), Rotation2d::interpolate);
        // Input: distance to target in meters, Output: flywheel RPM
        private final static InterpolatingDoubleTreeMap hubFlywheelMap = new InterpolatingDoubleTreeMap();
        // Input: distance to target in meters, Output: time of flight in seconds
        private final static InterpolatingDoubleTreeMap hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();

        // Input: distance to target in meters, Output: hood angle (Rotation2d)
        private final static InterpolatingTreeMap<Double, Rotation2d> feedHoodAngleMap = new InterpolatingTreeMap<>(
                        InverseInterpolator.forDouble(), Rotation2d::interpolate);
        // Input: distance to target in meters, Output: flywheel RPM
        private final static InterpolatingDoubleTreeMap feedFlywheelMap = new InterpolatingDoubleTreeMap();
        // Input: distance to target in meters, Output: time of flight in seconds
        private final static InterpolatingDoubleTreeMap feedTimeOfFlightMap = new InterpolatingDoubleTreeMap();

        static {
                for (double[] shotData : Constants.TurretConstants.SHOT_MAP) {
                        double distanceMeters = shotData[0];
                        double hoodAngleDegrees = shotData[1];
                        double flywheelRPM = shotData[2];
                        double timeOfFlightSeconds = shotData[3];

                        // Populate the interpolating maps
                        hubHoodAngleMap.put(distanceMeters, Rotation2d.fromDegrees(hoodAngleDegrees));
                        hubFlywheelMap.put(distanceMeters, flywheelRPM);
                        hubTimeOfFlightMap.put(distanceMeters, timeOfFlightSeconds);
                }

                for (double[] shotData : Constants.TurretConstants.FEED_SHOT_MAP) {
                        double distanceMeters = shotData[0];
                        double hoodAngleDegrees = shotData[1];
                        double flywheelRPM = shotData[2];
                        double timeOfFlightSeconds = shotData[3];

                        // Populate the interpolating maps
                        feedHoodAngleMap.put(distanceMeters, Rotation2d.fromDegrees(hoodAngleDegrees));
                        feedFlywheelMap.put(distanceMeters, flywheelRPM);
                        feedTimeOfFlightMap.put(distanceMeters, timeOfFlightSeconds);
                }
        }

        private static ShotSolution calculate(InterpolatingDoubleTreeMap tofMap, InterpolatingDoubleTreeMap flywheelMap,
                        InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap,
                        Pose2d turretPosition, Translation2d targetPosition, ChassisSpeeds robotVelocity) {
                double distanceToTarget = turretPosition.getTranslation().getDistance(targetPosition);
                double timeOfFlight = tofMap.get(distanceToTarget);
                double vx = -robotVelocity.omegaRadiansPerSecond * (Constants.TurretConstants.TURRET_TRANSFORM_3D.getY());
                double vy = robotVelocity.omegaRadiansPerSecond * (Constants.TurretConstants.TURRET_TRANSFORM_3D.getX());
                Translation2d tangentialVelocity = new Translation2d(vx, vy).rotateBy(turretPosition.getRotation());
                Logger.recordOutput("ShotCalculator/TangentialVelocity", tangentialVelocity);
                Translation2d turretVelocity = new Translation2d(
                                robotVelocity.vxMetersPerSecond + tangentialVelocity.getX(),
                                robotVelocity.vyMetersPerSecond + tangentialVelocity.getY());
                for (int i = 0; i < 20; i++) { // Numerically solve differential equation TODO: find # of iterations
                                               // that
                                               // converges best
                        Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                                        turretVelocity.getX(),
                                        turretVelocity.getY()).times(-timeOfFlight));
                        distanceToTarget = turretPosition.getTranslation().getDistance(predictedTarget);
                        timeOfFlight = tofMap.get(distanceToTarget);
                }
                Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                                turretVelocity.getX(),
                                turretVelocity.getY()).times(-timeOfFlight));
                Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlight);
                Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(predictedTarget, new Rotation2d()));
                distanceToTarget = turretPosition.getTranslation().getDistance(predictedTarget);
                Rotation2d hoodAngle = hoodAngleMap.get(distanceToTarget);
                double flywheelRPM = flywheelMap.get(distanceToTarget);
                Rotation2d turretAngle = predictedTarget.minus(turretPosition.getTranslation()).getAngle();
                ShotSolution solution = new ShotSolution(
                                hoodAngle, flywheelRPM, turretAngle, distanceToTarget, timeOfFlight);
                solution.setRobotVelocity(robotVelocity);
                return solution;

        }

        public static ShotSolution calculateHubShot(Pose2d turretPosition, Translation2d targetPosition,
                        ChassisSpeeds robotVelocity) {
                return calculate(hubTimeOfFlightMap, hubFlywheelMap, hubHoodAngleMap, turretPosition, targetPosition,
                                robotVelocity);
        }

        public static ShotSolution calculateFeedShot(
                        Pose2d turretPosition, Translation2d targetPosition,
                        ChassisSpeeds robotVelocity) {
                return calculate(feedTimeOfFlightMap, feedFlywheelMap, feedHoodAngleMap, turretPosition, targetPosition,
                                robotVelocity);
        }
}