// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends PIDCommand {

  public DriveTrain m_drive;

  /** Creates a new AutoBalance. */
  public AutoBalance(DriveTrain drive,double p, double i, double d,double deadband) 
  {
    super(
      new PIDController(p, i, d),
      drive::getPitch,
      0.0,
      output -> {drive.driveRaw(0, -output);}
    );

    m_drive = drive;

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(deadband,4);
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.enableMotorBreak();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.disableMotorBreak();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
