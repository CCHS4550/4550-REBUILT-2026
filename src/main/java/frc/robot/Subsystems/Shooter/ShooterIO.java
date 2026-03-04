package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

interface ShooterIO {
    void updateInputs();

    Rotation2d getHoodAngle();

    Rotation2d getTurretAngle();

    double getFlywheelRPM();

    void moveHoodToAngle(Rotation2d angle);

    void setTurretAngle(double angle);

    void setFlywheelRPM(double rpm);

    void setFlywheelPercent(double percent);

    double getRelativeTurretAngleRadians();

    void setHoodAngle(Rotation2d angle);

    void zeroTurretToEncoder();

    double getFlywheelCurrent();

    double getFlywheelAcceleration();

    double getHoodCurrent();

    double getTurretCurrent();

    Rotation2d getTurretAngleSetpointForTrajectory(Translation3d _trajectorySetpoint);

    Rotation2d getFutureSetpointEstimate(Rotation2d currentSetpoint,
                                        double driveAngularVelocity, double foresightTime);

                                        Rotation2d hoodAngleToMotorAngle(Rotation2d hoodAngle);
                                        Rotation2d motorAngleToHoodAngle(Rotation2d motorAngle);
                                        Rotation2d launchAngleToHoodAngle(Rotation2d launchAngle, double rpm);
                                        double shooterMPSToRPM(double mps);
                                        Rotation2d getHoodAngleSetpointForTrajectory(Translation3d trajectory);
                                        double getFlywheelRPMSetpointForTrajectory(Translation3d _trajectorySetpoint);

}