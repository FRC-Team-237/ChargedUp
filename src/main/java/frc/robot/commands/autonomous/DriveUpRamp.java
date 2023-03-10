// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Debouncer m_debouncer;
  private double m_time; 
  /** Creates a new DriveUpRamp. */
  public DriveUpRamp(DriveTrain drive, Direction dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive; 
    m_dir = dir; 

    m_debouncer = new Debouncer(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Capture current pitch and reset state 
    m_startingPitch = m_drive.getPitch(); 
    m_state = States.DRIVE2RAMP; 
    SmartDashboard.putString("Current Climb State", "Driving to Ramp"); 
    // m_time = getFPGA
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state){
      case DRIVE2RAMP:
      m_drive.driveRaw(0, (m_dir == Direction.FORWARD) ? -0.35 : 0.35);
      if (false) {
        m_startingPitch = m_drive.getPitch(); 
        m_state = States.CLIMB;
        SmartDashboard.putString("Current Climb State", "Climbing Ramp"); 
      }
      break; 
      case CLIMB:
        m_drive.driveRaw(0, (m_dir == Direction.FORWARD) ? -0.25 : 0.25);
        if(m_debouncer.calculate(Math.abs(m_drive.getPitch() - m_startingPitch) > 5)) {
          m_state = States.LEVEL; 
          SmartDashboard.putString("Current Climb State", "LEVEL!!!"); 
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
