package frc.robot.Auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constant.FieldConstants;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.WantedSuperstructureState;
import frc.robot.RobotContainer;
import frc.robot.Robotstate;

/** A factory for creating autonomous programs for a given {@link Auto} */
class AutoFactory {
  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;

  private final Superstructure superstructure;

  private final Choreo.TrajectoryCache trajectoryCache;

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer, Superstructure superstructure) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
    this.superstructure = superstructure;

    trajectoryCache = new Choreo.TrajectoryCache();
  }

  /* Autonomous program factories
   *
   * Factory methods should be added here for each autonomous program.
   * The factory methods must:
   *   1. Be package-private (i.e. no access modifier)
   *   2. Accept no parameters
   *   3. Return a link Command
   */
  private static final Command IDLE_COMMAND = Commands.idle();

  Pair<Pose2d, Command> createIdleCommand() {
    return Pair.of(FieldConstants.getMiddleStartingPose(), IDLE_COMMAND);
  }

  Pair<Pose2d, Command> createTestAuto() {
    System.out.println(trajectoryName(Location.FAR_LEFT_STARTING, Location.LEFT_TRENCH));
    var initialTrajectory =
        trajectoryCache
            .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.LEFT_TRENCH))
            .get();

    var initialPose = initialTrajectory.getInitialPose(false).get();
    return Pair.of(
        initialPose,
        Commands.sequence(
            new InstantCommand(
                () -> robotContainer.getSwerveSubsystem().resetTranslationAndRotation(initialPose)),
            followTrajectory((Trajectory<SwerveSample>) initialTrajectory)));
  }

  Pair<Pose2d, Command> createLeftTrenchToNeutralZoneToRightTrenchShoot(){
    System.out.println(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH));
    var trajPart1 = 
      trajectoryCache
        .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH)+"_part0")
        .get();
     var trajPart2 = 
      trajectoryCache
        .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH)+"_part1")
        .get();
    var trajPart3 = 
      trajectoryCache
        .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH)+"_part2")
        .get();
    var trajPart4 = 
      trajectoryCache
        .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH)+"_part3")
        .get();
    var trajPart5 = 
      trajectoryCache
        .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.NEUTRAL_ZONE, Location.RIGHT_TRENCH)+"_part4")
        .get();

    var initialPose = trajPart1.getInitialPose(false).get(); // this is false because we have two separate pathings for each side
    return Pair.of(
        initialPose,
        Commands.sequence(
          followTrajectory((Trajectory<SwerveSample>) trajPart1),
          Commands.parallel(
            followTrajectory((Trajectory<SwerveSample>) trajPart2)),
            new InstantCommand(()->superstructure.setWantedSuperstructureState(WantedSuperstructureState.EXTEND_INTAKE, false)),
          Commands.parallel(
            followTrajectory((Trajectory<SwerveSample>) trajPart3),
            new InstantCommand(()->superstructure.setWantedSuperstructureState(WantedSuperstructureState.EXTEND_INTAKE, true))
          ),
          Commands.parallel(
            followTrajectory((Trajectory<SwerveSample>) trajPart4),
            new InstantCommand(()-> superstructure.setWantedSuperstructureState(WantedSuperstructureState.EXTEND_INTAKE, true))
          ),
          followTrajectory((Trajectory<SwerveSample>) trajPart5),
          new InstantCommand(()-> superstructure.setWantedSuperstructureState(WantedSuperstructureState.ACTIVE_SHOOT, false)))
        );
      
    
  }

  Pair <Pose2d, Command> createShootPreload(){
    return Pair.of(
      Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry(), new InstantCommand(()-> superstructure.setWantedSuperstructureState(WantedSuperstructureState.ACTIVE_SHOOT, false)));
  }


  // Command setState(Superstructure.WantedSuperState state) {
  //     return robotContainer.getSuperstructure().setStateCommand(state);
  // }

  Command followTrajectory(Trajectory<SwerveSample> trajectory) {
    return new InstantCommand(
        () -> robotContainer.getSwerveSubsystem().setDesiredChoreoTrajectory(trajectory));
  }

  Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
    return new InstantCommand(
            () ->
                robotContainer
                    .getSwerveSubsystem()
                    .setDesiredPoseForDriveToPointWithConstraints(
                        point, maxVelocityOutputForDriveToPoint, 1.0))
        .andThen(
            new WaitUntilCommand(
                () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
  }

  Command driveToPointWithUnconstrainedMaxVelocity(
      Pose2d point, double maxVelocityOutputForDriveToPoint) {
    return new InstantCommand(
            () ->
                robotContainer
                    .getSwerveSubsystem()
                    .setDesiredPoseForDriveToPointWithConstraints(
                        point, maxVelocityOutputForDriveToPoint, Double.NaN))
        .andThen(
            new WaitUntilCommand(
                () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
  }

  // private String trajectoryName(final Location start, final Location end) {
  //   var name = "%S_TO_%S".formatted(start, end);
  //   var x = "%S_%S".formatted(alliance, name);
  //   System.out.println("%S_%S".formatted(alliance, name));
  //   return x;
  // }
  private String trajectoryName(Location... locations) {
    String joinedLocations =
        java.util.Arrays.stream(locations)
            .map(Location::name)
            .reduce((a, b) -> a + "_TO_" + b)
            .orElse("");

    return "%s_%s".formatted(alliance, joinedLocations);
}
}
