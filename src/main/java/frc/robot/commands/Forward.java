// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class Forward extends CommandBase {

  private double distance;
  private final RomiDrivetrain m_db;

  /** Creates a new Forward. */
  public Forward(RomiDrivetrain db, double inches) {
    m_db = db;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_db);
    distance = inches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.arcadeDrive(0,0);
    m_db.resetEncoders();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //first argument forward
    //second argument turn
    m_db.arcadeDrive(0.5,0);
    System.out.println(m_db.getLeftDistanceInch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_db.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(m_db.getAverageDistanceInch()) >= distance ;
  }
}
