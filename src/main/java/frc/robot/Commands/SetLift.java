package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Lift;

public class SetLift extends Command
{
    private Lift m_lift;
    private double speed;

    public SetLift(double speed)
    {
        m_lift = Lift.getInstance();
        this.speed = speed;

        addRequirements(m_lift);
    }

    @Override
    public void execute()
    {
        m_lift.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_lift.setSpeed(0);
    }
}
