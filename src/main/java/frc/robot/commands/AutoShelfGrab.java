// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimeLightInterface;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Stinger.GrabberState;

public class AutoShelfGrab extends CommandBase {
  private boolean finished;
  private Stinger m_stinger;
  private DriveTrain m_driveTrain;

  private double dx;
  private double dy;
  private double dz;
  private double speed;

  private LimeLightInterface armLimeLight;

  /** Creates a new AutoShelfGrab. */
  public AutoShelfGrab(Stinger stinger, DriveTrain driveTrain) {
    m_stinger = stinger;
    m_driveTrain = driveTrain;
    addRequirements(m_stinger, m_driveTrain);

    armLimeLight = new LimeLightInterface("limelight-arm");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    if(!armLimeLight.hasTarget()) finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished) {
      boolean hasTarget = armLimeLight.hasTarget();
      if(hasTarget && armLimeLight.getTHor() < 110) {

        double adjustedRotation = armLimeLight.getTX() / 45;
        adjustedRotation = Math.pow(Math.abs(adjustedRotation), 1.4) * Math.signum(armLimeLight.getTX());
        
        m_driveTrain.driveRaw(adjustedRotation, -0.25);

      } else if(hasTarget) {
        m_stinger.setGrabber(GrabberState.PINCH);
        finished = true;
      } else {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      new WaitCommand(0.75)
        .andThen(new ElbowToPosition(m_stinger, 38)).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
