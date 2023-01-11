// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;

public class AutoGrabbyCommand extends CommandBase {

  private enum states {
    START,
    LOWERARMS,
    CLOSEGRABBER,
    RAISEARMS,
    OPENGRABBER,
    END
  }
  private states currentState;
  private double timestamp;
  private Grabber m_grabber;
  private Shooter m_shooter;

  /** Creates a new AutoGrabbyCommand. */
  public AutoGrabbyCommand(Grabber grabber, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    timestamp = 0;
    m_grabber = grabber;
    m_shooter = shooter;
    addRequirements(m_grabber, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = states.START;
    m_grabber.setIsRedAlliance(DriverStation.getAlliance() == Alliance.Red);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isBallReady() && m_grabber.ballAtGrabber()) {
      Leds.getInstance().setColor(Constants.Colors.kTwoBalls);
    } else if (m_grabber.ballAtGrabber()) {
      Leds.getInstance().setColor(Constants.Colors.kOneBall);
    } else if (Leds.getInstance().getColor() != Constants.Colors.kHang) {
      Leds.getInstance().resetColor();
    }
    switch(currentState) {
      case START: {
        currentState = states.LOWERARMS;
        timestamp = Timer.getFPGATimestamp();
        break;
      }
      case LOWERARMS: {
        m_grabber.lowerGrabber(true);
        if (Timer.getFPGATimestamp() > timestamp + 1 && m_grabber.canPickup()) {
          currentState = states.CLOSEGRABBER;
          timestamp = Timer.getFPGATimestamp();
        }
        break;
      }
      case CLOSEGRABBER: {
        m_grabber.grabbyGrab(true);
        // if (!m_shooter.isBallReady()) {
        //   Leds.getInstance().setColor(Constants.Colors.kOneBall);
        // } else {
        //   Leds.getInstance().setColor(Constants.Colors.kTwoBalls);
        // }
        if (Timer.getFPGATimestamp() > timestamp + 0.5) {
          currentState = states.RAISEARMS;
          timestamp = Timer.getFPGATimestamp();
        }
        break;
      }
      case RAISEARMS: {
        m_grabber.lowerGrabber(false);
        if (m_grabber.grabberRaised() || Timer.getFPGATimestamp() > timestamp + 0.9) {
          currentState = states.OPENGRABBER;
        }
        break;
      }
      case OPENGRABBER: {
        if (!m_shooter.isBallReady()) {
          m_grabber.grabbyGrab(false);
          // Leds.getInstance().setColor(Constants.Colors.kOneBall);
        } else {
          // Leds.getInstance().setColor(Constants.Colors.kTwoBalls);
        }
        currentState = states.END;
        break;
      }
      case END: {}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_grabber.grabbyGrab(false);
    m_grabber.lowerGrabber(false);
    if (m_shooter.isBallReady() && m_grabber.ballAtGrabber()) {
      Leds.getInstance().setColor(Constants.Colors.kTwoBalls);
    } else if (Leds.getInstance().getColor() != Constants.Colors.kHang) {
      Leds.getInstance().resetColor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == states.END;
  }
}
