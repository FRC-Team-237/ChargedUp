// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;


public class EjectBallCommand extends CommandBase {
  /** Creates a new EjectBallCommand. */
  private Shooter m_sShooter;
  private Grabber m_gGrabber;
  public EjectBallCommand(Shooter shooter, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sShooter = shooter;
    m_gGrabber = grabber;
    addRequirements(m_sShooter,m_gGrabber); 
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gGrabber.grabbyGrab(false);
    m_sShooter.raiseTOT(true);
    m_sShooter.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sShooter.offWheel();
    m_sShooter.raiseTOT(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
