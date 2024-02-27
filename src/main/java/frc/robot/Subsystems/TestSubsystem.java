package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware.SparkMaxU;

public class TestSubsystem extends SubsystemBase 
{
    private static TestSubsystem instance;

    private SparkMaxU motor;
    private double speed;

    public static TestSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new TestSubsystem(9);
        }

        return instance;
    }

    private TestSubsystem(int motorID)
    {
        motor = new SparkMaxU(motorID, MotorType.kBrushless);
        speed = 0;
    }

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    @Override
    public void periodic()
    {
        motor.set(speed);
    }
}
