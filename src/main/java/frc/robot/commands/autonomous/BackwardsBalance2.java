// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Stinger.GrabberState;

public class BackwardsBalance2 extends CommandBase {

  DriveTrain driveTrain;
  Stinger stinger;
  double angleSign = 0.0;

  /** Creates a new BackwardBalance2. */
  public BackwardsBalance2(DriveTrain driveTrain, Stinger stinger) {
    this.driveTrain = driveTrain;
    this.stinger = stinger;
    addRequirements(driveTrain, stinger);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrain.driveRaw(0, -0.75);
    // if(Timer.getFPGATimestamp() > timeStamp + 1.0) {
    //   driveTrain.driveRaw(0, -0.35);
    //   if(!hitAngle) {
    //     angleSign = Math.signum(driveTrain.getPitch());
    //     hitAngle = true;
    //   }
    // }
    new InstantCommand(() -> { stinger.setGrabber(GrabberState.PINCH); })
    .andThen(new AutoDriveCommand(driveTrain, -35000, 0.75))
      // .andThen(new AutoDriveCommand(driveTrain, -35000, 0.75))
      .andThen(new AutoDriveCommand(driveTrain, -55000, 0.35))
      // .andThen(new InstantCommand(() -> { angleSign = Math.signum(driveTrain.getPitch()); }))
      // .andThen(new RepeatCommand(new InstantCommand(() -> { driveTrain.driveRaw(0, 0.25); }))
      //   .until(() -> { return Math.signum(driveTrain.getPitch()) != angleSign; }))
      // .andThen(new AutoDriveCommand(driveTrain, 4500, 0.25))
      .andThen(new AutoBalance(driveTrain, 0.021, 0.0005, 0.025, 10, 0))
    .schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
