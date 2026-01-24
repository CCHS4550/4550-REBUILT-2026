package frc.robot.Subsystems.Turret.Shooter;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robotstate;

public class ShooterCalc {
    // this code fucking sucks
    ChassisSpeeds chassisSpeeds;
    Pose2d robotPose;
    Pose2d predictedRobotPose;
    Vector<N3> robotSpeed;
    Vector<N3> normalGeometryOutput;
    Vector<N3> finalOutput;
    double timeDelay;
    double flywheelSpeedMagnitude;
    double lateralDistance;
    double geometryVelocity;
    Pose2d desiredPose;

    public void initValues (){
        timeDelay = 0.03;
        chassisSpeeds = Robotstate.getInstance().getRobotChassisSpeeds();
        robotSpeed = VecBuilder.fill(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, 0.0);
        robotPose = Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry(); // this is sketch

        predictedRobotPose = robotPose.transformBy(new Transform2d(chassisSpeeds.vxMetersPerSecond * timeDelay, chassisSpeeds.vyMetersPerSecond * timeDelay, new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * timeDelay)));

        desiredPose = new Pose2d(0, 0, new Rotation2d(0)); // should be a constant that is the hub locationes

        
        lateralDistance = desiredPose.getTranslation().getDistance(predictedRobotPose.getTranslation());

        geometryVelocity = 6.7; // should be meters per second, not in a certain direction, convert to RPM for shooter flywheels later

        normalGeometryOutput = VecBuilder.fill(calculateElevationAngle().getRadians(), calculateRotationAngle().getRadians(), geometryVelocity);

        finalOutput = normalGeometryOutput.plus(robotSpeed);


        

        
        

    }

    public Rotation2d calculateElevationAngle(){
        // gravity should be a constant (9.81)
        double elevationAngle = Math.atan2(lateralDistance + Math.sqrt(Math.pow(lateralDistance, 2) - 2*9.81*(90 /*hub height*/ - 80 /*turret height*/)*Math.pow(lateralDistance/geometryVelocity, 2)), (9.81 * lateralDistance)/Math.pow(geometryVelocity, 2));
        return new Rotation2d(elevationAngle);
    }

    public Rotation2d calculateRotationAngle (){
        double rotationAngle = Math.atan2(90/*hub y value*/ - 80/*turret y value*/, 600 /*hub x value*/ - 50 /*turret x value*/);
        return new Rotation2d(rotationAngle);
    }




}
