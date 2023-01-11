// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LongBall extends SequentialCommandGroup {
  /** Creates a new LongBall. */
  public LongBall(DriveTrain driveTrain, double angle, double distance, double distance2, Grabber grabber, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveCommand(driveTrain, distance, 0.4),
      new TurnToAngle(angle, driveTrain, 0.015, 0.0000, 0.0025, 4),
      new InstantCommand(driveTrain::resetEncoders, driveTrain),
      new AutoDriveCommand(driveTrain, distance2, 0.4),
      new ParallelCommandGroup(
        new TargetFinder(driveTrain, 0.3),
        new AutoGrabbyCommand(grabber, shooter)
      )
    );
  }
}
