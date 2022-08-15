// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensor.RomiGyro;
import frc.robot.Constants.PIDConstants;


/** An example command that uses an example subsystem. */
public class straightLinePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public straightLinePID(RomiDrivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_romiDriveTrain.resetEncoders();

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_leftEncoderData = RobotContainer.m_romiDriveTrain.m_leftEncoder.getRaw();
    double m_rightEncoderData = RobotContainer.m_romiDriveTrain.m_leftEncoder.getRaw();

    SmartDashboard.putNumber("gyro data", RobotContainer.m_romiDriveTrain.m_gyro.getAngle());

    PIDController m_pid = new PIDController(PIDConstants.P_DRIVE, PIDConstants.I_DRIVE, PIDConstants.D_DRIVE);
    m_subsystem.m_diffDrive.arcadeDrive(0.5, m_pid.calculate(m_subsystem.m_gyro.getAngleZ(),0)) ;



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
