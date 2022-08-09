// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Forward extends CommandBase {

  private double distance;


  /** Creates a new Forward. */
  public Forward(double inches) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_romiDriveTrain);
    distance = inches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_romiDriveTrain.resetEncoders();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //first argument forward
    //second argument turn
    RobotContainer.m_romiDriveTrain.arcadeDrive(0.15,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_romiDriveTrain.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return RobotContainer.m_romiDriveTrain.getLeftDistanceInch() >= distance ;
  }
}
