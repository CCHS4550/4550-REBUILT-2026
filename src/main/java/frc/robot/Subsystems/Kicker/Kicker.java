package frc.robot.Subsystems.Kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase
{
    private KickerIO kickerIO;

    private boolean atGoal;

    private KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

    public enum KickerSystemState
    {
        IDLE,
        KICK
    }

    public enum KickerWantedState
    {
        IDLE,
        KICK
    }

    private KickerSystemState systemState = KickerSystemState.IDLE;
    private KickerWantedState wantedState = KickerWantedState.IDLE;

    public Kicker(KickerIO io)
    {
        this.kickerIO = io;

        atGoal = true;
    }

    @Override
    public void periodic()
    {
        kickerIO.updateInputs(kickerInputs);

    }

    public KickerSystemState handleStateTransitions()
    {
        switch(wantedState)
        {
            case IDLE: return KickerSystemState.IDLE;
            case KICK: return KickerSystemState.KICK;
            default: return KickerSystemState.IDLE;
        }
    }

    public void applyStates()
    {
        switch(systemState)
        {
            case IDLE:
                atGoal = true;
                break;
            case KICK: 
                //kicking logic
                break;
            default: break;
        }
    }
}
