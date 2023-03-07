// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TargetConePID extends CommandBase {

  private DriveTrain driveTrain;
  private Debouncer rotateDebouncer; 
  private NetworkTable limelightTable; 
  private double tx;
  private double tv; 
  
  PIDController rotationPID; 

  /** Creates a new TargetConePID. */
  public TargetConePID(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    rotateDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    rotationPID = new PIDController(0.05, 0, 0.0025);
    rotationPID.setTolerance(2);
    addRequirements(driveTrain);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-arm");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = limelightTable.getEntry("tx").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);

    double rotation = -rotationPID.calculate(tx);
    rotation =  Math.min(Math.max(rotation, -0.25), 0.25);

    driveTrain.driveRaw(rotation, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (rotateDebouncer.calculate(rotationPID.atSetpoint()))
    || tv < 1;
  }
}
