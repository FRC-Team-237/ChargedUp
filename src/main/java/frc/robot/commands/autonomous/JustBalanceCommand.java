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

public class JustBalanceCommand extends CommandBase {

  private DriveTrain driveTrain;
  private double flatAngle = 0.0;
  private double rampAngle;

  /** Creates a new JustBalanceCommand. */
  public JustBalanceCommand(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.enableMotorBreak();
    new WaitCommand(0.5)
    .andThen(new InstantCommand(() -> { flatAngle = driveTrain.getPitch(); }))
    .andThen(new AutoDriveCommand(driveTrain, 20000, 0.75))

    .andThen(new WaitCommand(0.25))

    .andThen(new AutoDriveCommand(driveTrain, 25000, 0.35))
    .andThen(new InstantCommand(() -> { rampAngle = driveTrain.getPitch(); }))
    .andThen(new AutoDriveCommand(driveTrain, 100000, 0.15)
        .until(() -> { return Math.abs(driveTrain.getPitch() - rampAngle) > 4; }))

    .andThen(new AutoDriveCommand(driveTrain, -4500, 0.2))

    .andThen(new WaitCommand(0.25))

    .andThen(new AutoBalance(driveTrain, 0.025, 0.0005, 0.001, 5, flatAngle))
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
    return false;
  }
}
