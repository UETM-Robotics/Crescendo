package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsF.PortConstants;
import frc.robot.Hardware.SparkMaxU;

public class Lift extends SubsystemBase
{
    private static Lift instance;

    public static Lift getInstance()
    {
        if(instance == null)
        {
            instance = new Lift();
        }

        return instance;
    }

    private SparkMaxU motorL, motorR;

    private Lift()
    {
        motorL = new SparkMaxU(PortConstants.liftL, MotorType.kBrushless);
        motorR = new SparkMaxU(PortConstants.liftR, MotorType.kBrushless);

        motorR.follow(motorL);
    }

    public void setSpeed(double speed)
    {
        motorL.set(speed);
    }

    public double getSpeed()
    {
        return motorL.get();
    }
}
