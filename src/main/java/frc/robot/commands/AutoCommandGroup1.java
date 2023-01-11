// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommandGroup1 extends SequentialCommandGroup {
  private DriveTrain m_driveTrain;
  /** Creates a new AutoCommandGroup1. */
  public AutoCommandGroup1(Grabber grabber, DriveTrain drive, double turnAngle, double reverseDistance) {
    m_driveTrain = drive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new InstantCommand(m_driveTrain::resetEncoders, m_driveTrain),
      // new InstantCommand(m_driveTrain::resetHeading, m_driveTrain),
      
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new ScheduleCommand(
            new AutoGrabbyCommand(grabber)
          ),
          new BallAtGrabberCommand(grabber)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1),
          // new DelayCommand(1
          new TargetFinder(m_driveTrain, 0.3)
        )
      ) ,
      // new AutoDriveCommand(m_driveTrain, reverseDistance, 0.4) , // 30000 less that travelled ~150000
      new ParallelRaceGroup(
        new WaitCommand(2),
        // new DelayCommand(1),
        new TurnToAngle(
          turnAngle,
          m_driveTrain,
          0.015,
          0.0000,
          0.0025,
          4
        )
      ),
      new GoalFinder(drive, Constants.LimeLight.kGoalDriveP, true)
      // new ConditionalCommand(
      //   new GoalFinder(drive, Constants.LimeLight.kGoalDriveP, true),
      //   new InstantCommand(drive::resetEncoders, drive).andThen(new AutoDriveCommand(m_driveTrain, reverseDistance, 0.4)),
      //   NetworkTableInstance.getDefault().getTable("limelight-goal").getEntry("tv").getDouble(0) < 1.0
      // ),
    );
  }
}
