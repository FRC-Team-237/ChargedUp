// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveUpRamp extends CommandBase {

  enum States {
    DRIVE2RAMP,
    CLIMB,
    LEVEL
  }
  enum Direction {
    FORWARD,
    BACK
  }
  public States m_state; 
  private Direction m_dir; 
  private DriveTrain m_drive;
  private double m_startingPitch;  
  /** Creates a new DriveUpRamp. */
  public DriveUpRamp(DriveTrain drive, Direction dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive; 
    m_dir = dir; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Capture current pitch and reset state 
    m_startingPitch = m_drive.getPitch(); 
    m_state = States.DRIVE2RAMP; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state){
      case DRIVE2RAMP:
      m_drive.driveRaw(0, (m_dir == Direction.FORWARD) ? 0.25 : -0.25);
      if (Math.abs(m_drive.getPitch() - m_startingPitch) > 8){
        m_startingPitch = m_drive.getPitch(); 
        m_state = States.CLIMB;
      }
      break; 
      case CLIMB:
        m_drive.driveRaw(0, (m_dir == Direction.FORWARD) ? 0.25 : -0.25);
        if(Math.abs(m_drive.getPitch()) < 2) {
          m_state = States.LEVEL; 
        }
        break; 
      case LEVEL:
        m_drive.driveRaw(0.0, 0.0);
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
    {
      m_state = States.DRIVE2RAMP; 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_state == States.LEVEL)
    {
      return true; 
    }
    else {
      return false; 
    }
  }
}
