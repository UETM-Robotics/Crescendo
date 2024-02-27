package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class SetIntakePivot extends Command
{
    private Intake m_intake;
    private double speed;

    public SetIntakePivot(double speed)
    {
        m_intake = Intake.getInstance();
    }

    @Override
    public void execute()
    {
        m_intake.setPivot(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_intake.setPivot(0);
    }
}
