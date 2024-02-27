// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.ConstantsU.OperatorConstants;

import frc.robot.Commands.Autos;
import frc.robot.Commands.ExampleCommand;
import frc.robot.Commands.SetIntake;
import frc.robot.Commands.SetIntakePivot;
import frc.robot.Commands.SetLift;
import frc.robot.Commands.TestCommand;
import frc.robot.Subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final XboxController driverCont = ConstantsU.getInstance().getDriverController();
  //private final CommandXboxController m_driverController = ConstantsU.getInstance().getDriverController();
  private CommandPS4Controller m_driverController = ConstantsU.getInstance().getDriverController();

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("AutoX", new PathPlannerAuto("AutoX"));
    autoChooser.addOption("AutoY", new PathPlannerAuto("AutoY"));
    autoChooser.addOption("AutoR", new PathPlannerAuto("AutoR"));
    autoChooser.addOption("Uhhh", new PathPlannerAuto("Uhhh"));
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    /*
    //Lift
    m_driverController.axisGreaterThan(0, 0).whileTrue(new SetLift(0));
    m_driverController.axisGreaterThan(0, 0).whileTrue(new SetLift(-0));

    //Intake
    m_driverController.R1().whileTrue(new SetIntake(0));
    m_driverController.L1().whileTrue(new SetIntake(-0));

    //Intake Pivot
    m_driverController.circle().onTrue(new SetIntakePivot(0));
    m_driverController.cross().onTrue(new SetIntakePivot(-0));

    //Shooter
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
  }
}
