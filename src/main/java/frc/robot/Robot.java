// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Simulation.BatterySimulator;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.OperatingMode;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.SwerveModuleStateU;
import frc.robot.Util.Vec2d;
import frc.robot.Util.Vec3d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
    Drivetrain drivetrain = Drivetrain.getInstance();
    private Field2d m_field = new Field2d(); //WPI Lib class that handles and manages the simulated field

    CommandPS4Controller driverController = ConstantsU.getInstance().getDriverController();
    
    private Command m_autonomousCommand;

    RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
      m_robotContainer = new RobotContainer();
      
      SmartDashboard.putData("Field", m_field); //Upload field to SmartDashboard
      drivetrain.resetHeading();
      drivetrain.resetOdometry( new Vec3d(2.0, 7.0, 0) ); //Resets the Robot's position.
                                                            //Odometry (X,Y) = (0,0) is the bottom left corner of the field
                                                            //Rotations are Counter-Clockwise Positive
                                                            //This initial position describes a robot 3 meters from the bottom edge,
                                                            //                                        3 meters from the driver station,
                                                            //                                        Rotated 90 degrees Clockwise
    }

    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      drivetrain.updateOdometry();
      m_field.setRobotPose(drivetrain.getRobotPosition().asPose2d());

      SmartDashboard.putNumber("X", drivetrain.getRobotPosition().X());
      SmartDashboard.putNumber("Y", drivetrain.getRobotPosition().Y());
      SmartDashboard.putNumber("R", drivetrain.getRobotPosition().Theta().getDegrees());

      CommandScheduler.getInstance().run();

      PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        SmartDashboard.putNumber("pX", pose.getX());
        SmartDashboard.putNumber("pY", pose.getY());
        SmartDashboard.putNumber("pR", pose.getRotation().getDegrees());
      });

      PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        SmartDashboard.putNumber("desired pX", pose.getX());
        SmartDashboard.putNumber("desired pY", pose.getY());
        SmartDashboard.putNumber("desired pR", pose.getRotation().getDegrees());
      });

      SmartDashboard.putNumber("FR D Rot", drivetrain.getModules()[1].getThrottle().getEncoder().getPosition());
      SmartDashboard.putNumber("Speed X", drivetrain.getChassisSpeeds().vxMetersPerSecond);
    }

    @Override
    public void autonomousInit()
    {
      for(SwerveModule i : drivetrain.getModules())
      {
        i.getThrottle().getEncoder().setPosition(0);
      }

      //drivetrain.resetOdometry( new Vec3d(2.0, 7.0, 0) );
      //drivetrain.resetHeading();

      //drivetrain.resetPose(new Pose2d(2, 7, Rotation2d.fromDegrees(0)));

      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if(m_autonomousCommand != null)
      {
        m_autonomousCommand.schedule();
      }
    }

    @Override
    public void autonomousPeriodic()
    {
      //drivetrain.set(new ChassisSpeeds(0.1, 0, 0));
    }

    @Override
    public void teleopInit()
    {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }

    @Override
    public void teleopPeriodic()
    {
      double x, y, t;

      x = Math.abs(driverController.getLeftY()) < 0.1 ? 0 : -driverController.getLeftY() * RobotConstants.translationScale;
      y = Math.abs(driverController.getLeftX()) < 0.1 ? 0 : -driverController.getLeftX() * RobotConstants.translationScale;
      t = Math.abs(driverController.getRightX()) < 0.1 ? 0 : driverController.getRightX() * RobotConstants.rotateScale;

      drivetrain.set(new ChassisSpeeds(x, y, t), false);
    }

    @Override
    public void disabledInit() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , false);
    }

    @Override
    public void disabledPeriodic() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , false);
    }

    double testStartTime = 0;

    @Override
    public void testInit() {
      testStartTime = RobotController.getFPGATime();
    }

    @Override
    public void testPeriodic()
    {
      drivetrain.set( new ChassisSpeeds(0, 0.0, 0.0));
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {

      BatterySimulator.getInstance().updateSimulatedBatteryVoltage();

      drivetrain.updateSimModules();

      ChassisSpeeds speed = drivetrain.getChassisSpeeds();

      Vec2d xy_velocity = new Vec2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

      SmartDashboard.putNumber("speed", xy_velocity.len());
      SmartDashboard.putNumber("turn speed", speed.omegaRadiansPerSecond);
    }
}
