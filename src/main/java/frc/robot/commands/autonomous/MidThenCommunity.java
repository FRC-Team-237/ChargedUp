// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DrivePosition;
import frc.robot.commands.ElbowToPosition;
import frc.robot.commands.ExtendToPosition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Pincher.DropState;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Stinger.GrabberState;
import frc.robot.subsystems.Stinger.ShoulderState;

public class MidThenCommunity extends CommandBase {

  DriveTrain driveTrain;
  Stinger stinger;
  Pincher pincher;
  /** Creates a new MidThenCommunity. */
  public MidThenCommunity(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
    this.driveTrain = driveTrain;
    this.stinger = stinger;
    this.pincher = pincher;
    addRequirements(driveTrain, stinger, pincher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    new InstantCommand(() -> {
      stinger.setShoulder(ShoulderState.RAISED);
      stinger.setGrabber(GrabberState.PINCH);
      pincher.setDropper(DropState.LOWERED);
    })
    .andThen(new WaitCommand(1))
    .andThen(
    new InstantCommand(() -> {
      driveTrain.enableMotorBreak();
    }))
    .andThen(new WaitCommand(0.15))

    .andThen(new ElbowToPosition(stinger, 38))
    .andThen(new ExtendToPosition(stinger, 650))
    .andThen(new WaitCommand(0.2))
    .andThen(new InstantCommand(() -> { pincher.setDropper(DropState.RAISED); }))

    .andThen(new WaitCommand(1.2))
    .andThen(new ElbowToPosition(stinger, 32))
    .andThen(new WaitCommand(0.25))
    .andThen(new InstantCommand(() -> { stinger.setGrabber(GrabberState.DROP); }))

    .andThen(new WaitCommand(0.25))
    .andThen(new AutoDriveCommand(driveTrain, -5000, 0.3))
    
    .andThen(new DrivePosition(stinger, pincher))
    .andThen(new AutoDriveCommand(driveTrain, -7500, 0.35))
    .andThen(new WaitCommand(0.5))
    .andThen(new InstantCommand(() -> { driveTrain.disableMotorBreak(); }))
    .andThen(new AutoDriveCommand(driveTrain, -154000, 0.35))
    .schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
