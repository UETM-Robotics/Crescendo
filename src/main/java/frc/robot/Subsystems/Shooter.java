package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsU;
import frc.robot.ConstantsF.PortConstants;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Hardware.SparkMaxU;

public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    public static Shooter getInstance()
    {
        if(instance == null)
        {
            instance = new Shooter();
        }

        return instance;
    }

    private SparkMaxU motorL, motorR, motorPiv;
    private PIDController pid;

    private double speedL;
    private double speedR;
    private double pivPos;

    private Shooter()
    {
        motorL = new SparkMaxU(PortConstants.shooterMotorL, MotorType.kBrushless);
        motorR = new SparkMaxU(PortConstants.shooterMotorR, MotorType.kBrushless);
        motorPiv = new SparkMaxU(PortConstants.shooterPiv, MotorType.kBrushless);

        pid = new PIDController(0, 0, 0);

        speedL = 0;
        speedR = 0;
        pivPos = 0;
    }

    public void setSpeeds(double speedL, double speedR)
    {
        this.speedL = speedL;
        this.speedR = speedR;
    }

    public void setPivPos(double pos)
    {
        pivPos = pos;
    }

    public void setFromAng(double angle)
    {
        double pos = angle / (2 * Math.PI) / RobotConstants.shooterGearRatio;
        setPivPos(pos);
    }

    public double calculateAngle()
    {
        Drivetrain drivetrain = Drivetrain.getInstance();

        double x = drivetrain.getRobotPose().getX();
        double y = drivetrain.getRobotPose().getY();
        
        double dist = Math.sqrt(x*x + y*y);

        return Math.atan2(RobotConstants.shootTospeaker, dist);
    }

    public void update()
    {
        motorL.set(speedL);
        motorR.set(speedR);

        pid.setSetpoint(pivPos);
        motorPiv.set(pid.calculate(motorPiv.getPosition()));
    }
}
