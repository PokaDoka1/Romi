// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensor.RomiGyro;
/** An example command that uses an example subsystem. */
public class straightLine extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_db;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public straightLine(RomiDrivetrain db) {
    m_db = db;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_leftEncoderData = m_db.m_leftEncoder.getRaw();
    double m_rightEncoderData = m_db.m_leftEncoder.getRaw();

    SmartDashboard.putNumber("gyro data", m_db.m_gyro.getAngle());

    double REDUCTION = 0.015;
    m_db.m_diffDrive.arcadeDrive(0.5, -m_db.m_gyro.getAngleZ() * REDUCTION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
