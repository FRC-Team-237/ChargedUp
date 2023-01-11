// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class GoalFinder extends CommandBase {
  private DriveTrain m_dDriveTrain; 
  private boolean m_hasTarget; 
  private boolean m_atTarget;
  private double m_driveP;
  private Debouncer m_SteerDebounce;
  private Debouncer m_DriveDebounce;
  private boolean m_checkForTarget;
  /** Creates a new GoalFinder. */
  public GoalFinder(DriveTrain drive, double driveP, boolean checkForTarget) {
    m_dDriveTrain = drive;
    m_driveP = driveP;
    m_SteerDebounce = new Debouncer(0.25, DebounceType.kBoth);
    m_DriveDebounce = new Debouncer(0.25, DebounceType.kBoth);
    m_checkForTarget = checkForTarget;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dDriveTrain);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasTarget = false; 
    m_atTarget = false; 
    m_dDriveTrain.enableMotorBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight-goal").getEntry("tv").getDouble(0); 
    double tx = NetworkTableInstance.getDefault().getTable("limelight-goal").getEntry("tx").getDouble(0); 
    double ty = NetworkTableInstance.getDefault().getTable("limelight-goal").getEntry("ty").getDouble(0); 
  
    
    SmartDashboard.putNumber("Shooter tx", tx);
    SmartDashboard.putNumber("Shooter ty", ty);

    if (tv < 1.0)
    {
      m_hasTarget = false; 
      return; 
    }
    m_hasTarget = true; 

    

    double turnValue = tx * Constants.LimeLight.kGoalSteerP; 
    // double turnValue = Math.signum(tx) * 0.25;
    SmartDashboard.putNumber("Tracking Turn Value", turnValue);
    double driveValue = -ty * m_driveP; 
    // if (driveValue < Constants.LimeLight.kMinSpeed)
    //   m_atTarget = true; 
    // if (driveValue > Constants.LimeLight.kMaxDrive)
    //   driveValue = Constants.LimeLight.kMaxDrive; 
    // SmartDashboard.putNumber("Tracking Drive Value", driveValue);
    if (driveValue > Constants.LimeLight.kGoalMaxDrive) {
      m_dDriveTrain.driveRaw(turnValue, Constants.LimeLight.kGoalMaxDrive);
    } else {
      m_dDriveTrain.driveRaw(turnValue, driveValue);
    }
    
    m_atTarget = m_SteerDebounce.calculate(Math.abs(tx) <= 5) && m_DriveDebounce.calculate(Math.abs(ty) <= 3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dDriveTrain.drive(0, 0);
    m_dDriveTrain.disableMotorBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_hasTarget && m_checkForTarget)
    {
      return true; 
    }
    else if ((m_hasTarget || !m_checkForTarget) && m_atTarget)
    {
      return true; 
    }
    return false; 
  }
}
