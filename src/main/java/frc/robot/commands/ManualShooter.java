// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualShooter extends CommandBase {
  /** Creates a new ManualShooter. */
  
  private enum states {
    START,
    RAMPWHEEL,
    RAISETOT,
    LOWERTOT,
    RELEASEBALL,
    END
  }

  private states currentState;
  private double timestamp;
  private Shooter m_shooter;
  private Grabber m_grabber;

  public ManualShooter(Shooter shooter, Grabber grabber) {    
    timestamp = 0;
    m_shooter = shooter;
    m_grabber = grabber;
    addRequirements(m_shooter, m_grabber);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = states.START;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState) {
      case START: {
        
          currentState = states.RAMPWHEEL;
        
        break;
      }
      case RAMPWHEEL: {
        m_shooter.onWheel();
        if (m_shooter.getWheelSpeed1() > Constants.kShooter1SetPoint && m_shooter.getWheelSpeed2() > Constants.kShooter2SetPoint - 100) {
          timestamp = Timer.getFPGATimestamp();
          currentState = states.RAISETOT;
        }
        break;
      }
      case RAISETOT: {
        m_shooter.raiseTOT(true);
        if (Timer.getFPGATimestamp() > timestamp + 1) {
          timestamp = Timer.getFPGATimestamp();
          currentState = states.LOWERTOT;
        }
        break;
      }
      case LOWERTOT: {
        m_shooter.raiseTOT(false);
        if (Timer.getFPGATimestamp() > timestamp + 1) {
          
            currentState = states.END;
          
        }
        break;
      }
      case RELEASEBALL: {
        m_grabber.grabbyGrab(false);
        currentState = states.START;
        break;
      }
      case END: {}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.offWheel();
    m_shooter.raiseTOT(false);
    currentState = states.END;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == states.END;
  }
}
