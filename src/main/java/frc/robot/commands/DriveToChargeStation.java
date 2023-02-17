// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;



public class DriveToChargeStation extends CommandBase {

  private DriveTrain m_driveTrain;
  private double m_speed;
  /** Creates a new DriveToChargeStation. */
  public DriveToChargeStation(DriveTrain driveTrain, double speed) {

    m_driveTrain = driveTrain; 
    m_speed = speed;
    
    addRequirements(m_driveTrain);

    



    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.driveRaw(0, -m_speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getPitch())>10;
  }
}
