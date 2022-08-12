// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Forward;
import frc.robot.commands.Turn;
import frc.robot.commands.joystickDriving;
import frc.robot.commands.straightLine;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // static = access DriveTrain directly
  public static final RomiDrivetrain m_romiDriveTrain = new RomiDrivetrain();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDriveTrain);
  private final joystickDriving m_joystickDriving = new joystickDriving(m_romiDriveTrain);
  private final straightLine m_straightLine = new straightLine(m_romiDriveTrain);

  public static final PS4Controller PS4joystick = new PS4Controller(0);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("forward", new Forward(6));
    SmartDashboard.putData("turn 90", new Turn(90));
    SmartDashboard.putData("turn -90", new Turn(-90));

    
    //when in teleop, the m_joystickDriving is gonna run
    
    m_romiDriveTrain.setDefaultCommand(m_straightLine);
    new JoystickButton(PS4joystick, 1).whenPressed(new joystickDriving(m_romiDriveTrain));
  



  
    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
