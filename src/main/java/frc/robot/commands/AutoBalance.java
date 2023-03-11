// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.ConversionHelper;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends PIDCommand {

  public DriveTrain m_drive;
  private Debouncer balanceDebouncer;
  private double target;

  /** Creates a new AutoBalance. */
  public AutoBalance(DriveTrain drive,double p, double i, double d,double deadband, double target) 
  {
    super(
      new PIDController(p, i, d),
      drive::getPitch,
      target,
      output -> {drive.driveRaw(0, output);}
    );

    m_drive = drive;
    
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(deadband, 4);

    balanceDebouncer = new Debouncer(1);
    this.target = target;
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.enableMotorBreak();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //balanceDebouncer.calculate(Math.abs(m_drive.getPitch() - target) < 1);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    double output = getController().calculate(m_drive.getPitch() - target);
    output = ConversionHelper.clamp(output, -0.15, 0.15);
    m_drive.driveRaw(0.0, output);
  }
}
