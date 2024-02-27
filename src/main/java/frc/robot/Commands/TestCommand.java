package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TestSubsystem;

public class TestCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TestSubsystem sub;

    public TestCommand()
    {
        sub = TestSubsystem.getInstance();
        addRequirements(sub);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        sub.setSpeed(0.3);
    }

    @Override
    public void end(boolean interrupted)
    {
        sub.setSpeed(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
