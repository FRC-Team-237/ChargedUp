// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.DriveTrain;

public class BackwardsBalance extends CommandBase {

  DriveTrain driveTrain;
  private double angleSign = 0.0;

  /** Creates a new BackwardsBalance. */
  public BackwardsBalance(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new AutoDriveCommand(driveTrain, 1000, 0.75)
      .andThen(new AutoDriveCommand(driveTrain, -20000, 0.75)
      
      .andThen(new AutoDriveCommand(driveTrain, -10000, 0.35))
      .andThen(new AutoDriveCommand(driveTrain, -50000, 0.35))

      .andThen(new InstantCommand(() -> { angleSign = Math.signum(driveTrain.getPitch()); }))
      .andThen(
        new RepeatCommand(new InstantCommand(() -> { driveTrain.driveRaw(0, -0.75); }))
      ).until(() -> { return Math.signum(driveTrain.getPitch()) != angleSign; })

      .andThen(new AutoDriveCommand(driveTrain, 4500, 0.3))
      
      .andThen(new WaitCommand(0.25))
      .andThen(new AutoBalance(driveTrain, 0.025, 0.0005, 0.001, 5, 0))
    ).schedule();
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
    return false;
  }
}
