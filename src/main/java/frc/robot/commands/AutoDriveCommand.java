// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveCommand extends CommandBase {

  private DriveTrain m_driveTrain;
  private double m_distance;
  private boolean m_finished;
  private double m_encoderPosition;
  private double m_speed;
  private boolean m_forwards;

  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(DriveTrain driveTrain, double distance, double speed) {
    m_driveTrain = driveTrain;
    m_distance = distance;
    m_speed = speed;
    m_finished = false;
    m_forwards = distance >= 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_driveTrain.resetEncoders();
    m_encoderPosition = m_driveTrain.getEncPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_forwards) {
      m_driveTrain.driveRaw(0, m_speed);
      if (m_driveTrain.getEncPos() > m_encoderPosition + m_distance) {
        m_driveTrain.driveRaw(0, 0);
        m_finished = true;
      }
    } else {
      m_driveTrain.driveRaw(0, -m_speed);
      if (m_driveTrain.getEncPos() < m_encoderPosition + m_distance) {
        m_driveTrain.driveRaw(0, 0);
        m_finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
