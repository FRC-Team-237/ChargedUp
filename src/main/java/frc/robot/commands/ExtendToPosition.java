// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Stinger;

public class ExtendToPosition extends CommandBase {
  Stinger m_stinger;
  double m_position;

  /** Creates a new ExtendToPosition. */
  public ExtendToPosition(Stinger stinger, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_stinger = stinger;
    m_position = position;
    addRequirements(m_stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stinger.setExtendSetPoint(m_position);
    m_stinger.enableExtendClosedLoop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_stinger.setExtend(StingerDirection.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_stinger.isExtendFinished();
    return true;
  }
}
