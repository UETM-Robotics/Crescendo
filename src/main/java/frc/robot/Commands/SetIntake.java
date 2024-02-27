package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class SetIntake extends Command
{
    private Intake m_intake;
    private double speed;

    public SetIntake(double speed)
    {
        m_intake = Intake.getInstance();
        this.speed = speed;
    }

    @Override
    public void execute()
    {
        m_intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_intake.setSpeed(0);
    }
}
