package frc.robot.Subsystems;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Agitator.AgitatorSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.Turret.TurretSubsystem;
import frc.robot.Subsystems.Turret.TurretSubsystem.TurretSystemState;

public class Superstructure extends SubsystemBase

{
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final KickerSubsystem kickerSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final QuestNav questNav;
    private final AgitatorSubsystem agitatorSubsystem;

    public enum WantedSuperState {
        IDLE,
        UNDER_TRENCH,
        EXTEND_INTAKE_IDLE,
        EXTEND_INTAKE_ACTIVE,
        TRACKING_TARGET_PASSIVE_HUB,
        TRACKING_TARGET_PASSIVE_PASSING,
        ACTIVE_SHOOTING_HUB,
        ACTIVE_SHOOTING_PASSING,
        STOWED_INTAKE
    }

    public enum CurrentSuperState {
        IDLE,
        UNDER_TRENCH,
        EXTEND_INTAKE_IDLE,
        EXTEND_INTAKE_ACTIVE,
        TRACKING_TARGET_PASSIVE_HUB,
        TRACKING_TARGET_PASSIVE_PASSING,
        ACTIVE_SHOOTING_HUB,
        ACTIVE_SHOOTING_PASSING,
        STOWED_INTAKE
    }

    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;

    private boolean driveToPointActive = false;

    public Superstructure (
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        KickerSubsystem kicker,
        TurretSubsystem turret,
        QuestNav quest,
        AgitatorSubsystem agitator
    ){
        this.swerveSubsystem = swerve;
        this.intakeSubsystem = intake;
        this.kickerSubsystem = kicker;
        this.turretSubsystem = turret;
        this.questNav = quest;
        this.agitatorSubsystem = agitator;
    }

    @Override
    public void periodic(){
        currentSuperState = handleStateTransitions();
    }


    private CurrentSuperState handleStateTransitions (){
        CurrentSuperState returnCurrentSuperState;
        switch (wantedSuperState){
            default:
                returnCurrentSuperState = CurrentSuperState.IDLE;
                break;
            case IDLE:
                returnCurrentSuperState = CurrentSuperState.IDLE;
                break;
            case UNDER_TRENCH:
                returnCurrentSuperState = CurrentSuperState.UNDER_TRENCH;
                break;
            case EXTEND_INTAKE_IDLE:
                returnCurrentSuperState = CurrentSuperState.EXTEND_INTAKE_IDLE;
                break;
            case EXTEND_INTAKE_ACTIVE:
                returnCurrentSuperState = CurrentSuperState.EXTEND_INTAKE_ACTIVE;
                break;
            case TRACKING_TARGET_PASSIVE_HUB:
                returnCurrentSuperState = CurrentSuperState.TRACKING_TARGET_PASSIVE_HUB;
                break;
            case TRACKING_TARGET_PASSIVE_PASSING:
                returnCurrentSuperState = CurrentSuperState.TRACKING_TARGET_PASSIVE_PASSING;
                break;
            case ACTIVE_SHOOTING_HUB:
                returnCurrentSuperState = CurrentSuperState.ACTIVE_SHOOTING_HUB;
                break;       
            case ACTIVE_SHOOTING_PASSING:
                returnCurrentSuperState = CurrentSuperState.ACTIVE_SHOOTING_PASSING;
                break;
            case STOWED_INTAKE:
                returnCurrentSuperState = CurrentSuperState.STOWED_INTAKE;
                break;

        }
        return returnCurrentSuperState;
    }

    public void applyStates(){
        switch (currentSuperState){
            case UNDER_TRENCH:
                underTrench();
                break;
            case EXTEND_INTAKE_IDLE:
                extendIntakeIdle();
                break;
            case EXTEND_INTAKE_ACTIVE:
                extendIntakeActive();
                break;
            case TRACKING_TARGET_PASSIVE_HUB:
                trackingTargetPassingHub();
                break;
            case TRACKING_TARGET_PASSIVE_PASSING:
                trackingTargetPassivePassing();
                break;
            case ACTIVE_SHOOTING_HUB:
                activeShootingHub();
                break;
            case ACTIVE_SHOOTING_PASSING:
                activeShootingPassing();
                break;
            case STOWED_INTAKE:
                stowedIntake();
                break;
            case IDLE:
                robotIdle();
                break;
            default:
                robotIdle();
                break;

        }
    }

    public void assignWantedSuperState (WantedSuperState wantedSuperState){
        this.wantedSuperState = wantedSuperState;
    }

    public void activeShootingHub(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.SHOOT_SCORE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.SPINNING);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.RUNNING);
    }

    public void activeShootingPassing(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.PASS_TO_ALLIANCE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.SPINNING);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.RUNNING);
    }

    public void extendIntakeIdle(){
        intakeSubsystem.setWantedIntakeState(IntakeSubsystem.WantedIntakeState.EXTENDED_PASSIVE);
    }

    public void extendIntakeActive(){
        intakeSubsystem.setWantedIntakeState(IntakeSubsystem.WantedIntakeState.EXTENDED_INTAKING);
    }

    public void trackingTargetPassingHub(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.PASSIVE_HUB);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.IDLE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.IDLE);
    }

    public void trackingTargetPassivePassing(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.PASSIVE_PASS);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.IDLE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.IDLE);
    }

    public void stowedIntake(){
        intakeSubsystem.setWantedIntakeState(IntakeSubsystem.WantedIntakeState.STOWED);
    }

    public void underTrench(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.HOOD_ZEROED);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.IDLE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.IDLE);
    }
    public void robotIdle(){
        turretSubsystem.setWantedState(TurretSubsystem.TurretWantedState.IDLE);
        intakeSubsystem.setWantedIntakeState(IntakeSubsystem.WantedIntakeState.IDLE);
        agitatorSubsystem.setWantedState(AgitatorSubsystem.WantedState.IDLE);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.IDLE);
        kickerSubsystem.setWantedState(KickerSubsystem.KickerWantedState.IDLE);
    }
}
