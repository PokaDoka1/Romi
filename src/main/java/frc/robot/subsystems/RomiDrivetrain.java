// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensor.RomiGyro;
import frc.robot.commands.autoBackwards;

public class RomiDrivetrain extends SubsystemBase {
  //kCountsPerRevolution = relationship between encoder and how far wheel goes
  // 1 spin = how many encoder counts?
  public static final double kCountsPerRevolution = -1440.0;
  public static final double kWheelDiameterInch = 2.75591; // 70 mm

  public static RomiGyro m_gyro = new RomiGyro();

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  public final Spark m_leftMotor = new Spark(0);
  public final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  public final Encoder m_leftEncoder = new Encoder(4, 5, false);
  private final Encoder m_rightEncoder = new Encoder(6, 7, false);

  // Set up the differential drive controller
  public final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {

    SmartDashboard.putString("doggo", "doggo");
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();



    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch(){
    return(getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left distance", getLeftDistanceInch());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
