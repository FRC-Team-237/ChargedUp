// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TargetFinder extends CommandBase {
  private DriveTrain m_dDriveTrain; 
  private boolean m_hasTarget; 
  private boolean m_atTarget;
  private double m_driveP;
  /** Creates a new TargetFinder. */
  public TargetFinder(DriveTrain drive, double driveP) {
    m_dDriveTrain = drive;
    m_driveP = driveP;
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
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); 
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if (tv < 1.0)
    {
      m_hasTarget = false; 
      return; 
    }
    m_hasTarget = true; 

    double turnValue = tx * Constants.LimeLight.kSteerP; 
    SmartDashboard.putNumber("Tracking Turn Value", turnValue);
    double driveValue = m_driveP; 
    if (driveValue < Constants.LimeLight.kMinSpeed)
      m_atTarget = true; 
    if (driveValue > Constants.LimeLight.kMaxDrive)
      driveValue = Constants.LimeLight.kMaxDrive; 
    SmartDashboard.putNumber("Tracking Drive Value", driveValue);
    m_dDriveTrain.driveRaw(turnValue, driveValue);
    
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
    if (!m_hasTarget)
    {
      return true; 
    }
    else if (m_hasTarget && m_atTarget)
    {
      return true; 
    }
    return false; 
  }
}
