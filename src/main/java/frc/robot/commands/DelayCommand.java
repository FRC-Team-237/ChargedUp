// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayCommand extends CommandBase {

  private double m_delay;
  private double m_timestamp;
  private boolean m_started;

  /** Creates a new DelayCommand. Delay parameter is in seconds. */
  public DelayCommand(double delay) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_delay = delay;
    m_started = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timestamp = Timer.getFPGATimestamp();
    m_started = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_started && Timer.getFPGATimestamp() > m_timestamp + m_delay ;
  }
}
