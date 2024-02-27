package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsF.PortConstants;
import frc.robot.Hardware.SparkMaxU;

public class Intake extends SubsystemBase
{
    private static Intake instance;

    private double speed;
    private double pivSpeed;

    public static Intake getInstance()
    {
        if(instance == null)
        {
            instance = new Intake();
        }

        return instance;
    }

    private SparkMaxU motor, pivot;

    private Intake()
    {
        motor = new SparkMaxU(PortConstants.intakeMotor, MotorType.kBrushless);
        pivot = new SparkMaxU(PortConstants.intakePiv, MotorType.kBrushless);

        speed = 0;
        pivSpeed = 0;
    }

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    public void setPivot(double speed)
    {
        pivSpeed = speed;
    }

    public double getPivotPos()
    {
        return pivot.getPosition();
    }

    public void update()
    {
        motor.set(speed);
        pivot.set(pivSpeed);
    }
}
