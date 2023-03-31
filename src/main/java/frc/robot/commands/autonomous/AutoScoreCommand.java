// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Stinger;

public class AutoScoreCommand extends CommandBase {

  DriveTrain driveTrain;
  Stinger stinger;
  Pincher pincher;

  private boolean finished = false;
  private double rampAngle = 0.0;
  private double flatAngle = 0.0;

  /** Creates a new AutoScoreCommand. */
  public AutoScoreCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
    this.driveTrain = driveTrain;
    this.stinger = stinger;
    this.pincher = pincher;
    addRequirements(driveTrain, stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new InstantCommand(() -> {
    //   stinger.setGrabber(GrabberState.PINCH);
    //   pincher.setDropper(DropState.LOWERED);
    //   driveTrain.enableMotorBreak();
    // })
    // .andThen(new WaitCommand(0.15))

    // .andThen(new ElbowToPosition(stinger, 78.5))
    // .andThen(new WaitCommand(0.2))
    
    // .andThen(new ExtendToPosition(stinger, 210))
    // .andThen(new WaitCommand(0.2))

    // .andThen(new InstantCommand(() -> { pincher.setDropper(DropState.RAISED); }))
    // .andThen(new WaitCommand(0.35))

    // .andThen(new InstantCommand(() -> { stinger.setShoulder(ShoulderState.LOWERED); }))
    // .andThen(new WaitCommand(0.125))

// .andThen(new AutoDriveCommand(driveTrain, 25000, 0.25))
    // .andThen(new AutoDriveCommand(driveTrain, 15000, 0.25))
    // .andThen(new AutoDriveCommand(driveTrain, 10000, 0.15))
    // .andThen(new WaitCommand(0.5))
    // .andThen(new ElbowToPosition(stinger, 69))
    // .andThen(new WaitCommand(0.25))

    // .andThen(new InstantCommand(() -> { stinger.setGrabber(GrabberState.DROP); }))
    // .andThen(new WaitCommand(0.5))

    // .andThen(new AutoDriveCommand(driveTrain, -2000, 0.25))
    // .andThen(new DrivePosition(stinger, pincher))

// .andThen(new InstantCommand(() -> { driveTrain.disableMotorBreak(); }))

    new WaitCommand(0.5)
// .andThen(// P: 0.0042 I: 0.0008 D: 0.00014 1x
//   new TurnToAngle(180, driveTrain, 0.0044, 0.0009, 0.0012, 7.5))
    // .andThen(new TurnToAngle(90, driveTrain, 0.005, 0, 0, 30, 0.8, 0.125))
    // .andThen(new TurnToAngle(180, driveTrain, 0.005, 0.0008, 0.0007, 7.5, 1, 0.5))



    .andThen(new InstantCommand(() -> { flatAngle = driveTrain.getPitch(); System.out.println("FLAT PITCH: " + flatAngle); }))
    .andThen(new AutoDriveCommand(driveTrain, 20000, 0.75))
    .andThen(new WaitCommand(0.25))

    .andThen(new AutoDriveCommand(driveTrain, 25000, 0.35))
    .andThen(new InstantCommand(() -> { rampAngle = driveTrain.getPitch(); }))
    .andThen(
      new AutoDriveCommand(driveTrain, 100000, 0.15)
        .until(() -> { return Math.abs(driveTrain.getPitch() - rampAngle) > 4; }))

    .andThen(new AutoDriveCommand(driveTrain, -4500, 0.2))
    .andThen(new WaitCommand(0.25))
    .andThen(new AutoBalance(driveTrain, 0.025, 0.0005, 0.001, 5, flatAngle))
// .andThen(new WaitCommand(0.125))
// .andThen(new AutoDriveCommand(driveTrain, 10000, 0.75))
  // .until(() -> { return Math.abs(driveTrain.getPitch()) > 8; }))
// .andThen(new InstantCommand(() -> { System.out.println("ADJUSTED PITCH: " + (driveTrain.getPitch() - flatAngle)); }))
// .andThen(new AutoBalance(driveTrain, 0.025, 0.0005, 0.001, 5, flatAngle))
    .schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.disableMotorBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
