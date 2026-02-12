package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constant.Constants;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;

  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private final SysIdRoutine rotationToHeightConv;

  public enum ClimberWantedState {
    L1,
    L23,
    RETRACTED,
    AUTO_HOLD
  }

  public enum ClimberSystemState {
    L1,
    L23,
    RETRACTED,
    AUTO_HOLD
  }

  private boolean atGoal = true;
  private ClimberSystemState systemState = ClimberSystemState.RETRACTED;
  private ClimberWantedState wantedState = ClimberWantedState.RETRACTED;

  public Climber(ClimberIO io) {
    climberIO = io;

    rotationToHeightConv =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(climberIO::setVoltage, log -> {}, this));
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);

    atGoal = isAtTarget();

    systemState = handleStateTransitions();
    applyStates();
  }

  /* L3 climb sequence:
     RETRACTED ->
     L1 ->
     RETRACTED ->
     L23 ->
     RETRACTED ->
     L23 ->
     RETRACTED

     Auto climb sequence
     RETRACTED ->
     L1 ->
     AUTO_HOLD

     ...and decend sequence
     AUTO_HOLD ->
     L1->
     Move away ->
     RETRACTED

     Make sure to wait until the climber actually reaches the target height before changing the state
  */
  public boolean isAtTarget() {
    return MathUtil.isNear(climberInputs.climberHeightMeters, getDesiredHeight(), 0.25);
  }

  public void setState(ClimberWantedState state) {
    wantedState = state;
  }

  public double getDesiredHeight() {
    return switch (wantedState) {
      case L1 -> Constants.ClimberConstants.L1_HEIGHT;
      case L23 -> Constants.ClimberConstants.L2_3_HEIGHT;
      case AUTO_HOLD -> Constants.ClimberConstants.AUTO_HOLD_HEIGHT;
      case RETRACTED -> Constants.ClimberConstants.RETRACTED_HEIGHT;
      default -> Constants.ClimberConstants.RETRACTED_HEIGHT;
    };
  }

  public ClimberSystemState handleStateTransitions() {
    return switch (wantedState) {
      case L1 -> ClimberSystemState.L1;
      case L23 -> ClimberSystemState.L23;
      case AUTO_HOLD -> ClimberSystemState.AUTO_HOLD;
      case RETRACTED -> ClimberSystemState.RETRACTED;
      default -> ClimberSystemState.RETRACTED;
    };
  }

  public void applyStates() {
    switch (systemState) {
      case L1:
        climberIO.setHeight(Constants.ClimberConstants.L1_HEIGHT);
        // replace 1 with L1 height
        break;
      case L23:
        climberIO.setHeight(Constants.ClimberConstants.L2_3_HEIGHT);
        // replace 2 with L2/L3 height
        break;
      case RETRACTED:
        climberIO.setHeight(Constants.ClimberConstants.RETRACTED_HEIGHT);
        break;
      default:
        break;
        // the atGoal checks are probably cursed
    }
    ;
  }

  public boolean getAtGoal() {
    return atGoal;
  }

  public void assignWantedState (ClimberWantedState wantedState){
    this.wantedState = wantedState;
  }
}
