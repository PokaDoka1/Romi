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
public class straightLinePID2 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_db;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public straightLinePID2(RomiDrivetrain db) {

    m_db = db;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
  }

  //double setpoint = 0;
  public static final double kCountsPerRevolution = -1440.0;
  public static final double kWheelDiameterInch = 2.75591; // 70 mm
  private final double kDriveTick2Feet = (-(Math.PI * kWheelDiameterInch)/kCountsPerRevolution);
  final double kP = 0.25;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double m_leftEncoderData = m_db.m_leftEncoder.getRaw();
    //double m_rightEncoderData =m_db.m_leftEncoder.getRaw();

    //SmartDashboard.putNumber("gyro data", m_db.m_gyro.getAngle());
    /*
    if (RobotContainer.PS4joystick.getRawButtonPressed(1)){
      setpoint = 5;
    } else if (RobotContainer.PS4joystick.getRawButtonPressed(2)){
      setpoint = 0;
    }*/

    double setpoint = 20;

    //get sensor position
    //double sensorPosition = m_db.getLeftDistanceInch() * kDriveTick2Feet;
    double sensorPosition = -(m_db.getAverageDistanceInch());
    
    double error = setpoint - sensorPosition;
    //SmartDashboard.putNumber("error", error);
    System.out.println(error);

    double outputSpeed = kP * error;

    m_db.m_diffDrive.arcadeDrive(outputSpeed, 0);

    SmartDashboard.putNumber("encoder value", m_db.m_leftEncoder.get() * kDriveTick2Feet);
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
