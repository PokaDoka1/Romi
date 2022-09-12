// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RomiDrivetrain;

public class Turn extends CommandBase {
  
  //the inches variable dictates how far the romi travels
  //the turnSpeed variable is a parameter that dictates how fast to turn
  private double inches;
  private double turnSpeed;
  private final RomiDrivetrain m_db;

  /** Creates a new Forward. */
  public Turn(RomiDrivetrain db, double angle) {
    m_db = db;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_romiDriveTrain);
    inches = Math.abs(angle) + Math.PI * 5.25/ 360;
    turnSpeed = 0.1 + Math.signum(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_db.arcadeDrive(0,turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_db.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when isFInished or interuptted is done, goes to end
    return m_db.getLeftDistanceInch() >= inches ;
  }
}
