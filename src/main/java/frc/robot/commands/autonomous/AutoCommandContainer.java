// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DrivePosition;
import frc.robot.commands.ElbowToPosition;
import frc.robot.commands.ExtendToPosition;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Pincher.DropState;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Stinger.GrabberState;
import frc.robot.subsystems.Stinger.ShoulderState;

/** Add your docs here. */
public class AutoCommandContainer {
    private static AutoCommandContainer m_container = null; 
    public static AutoCommandContainer getInstance() {
      if (m_container == null) {
        m_container = new AutoCommandContainer(); 
      }
      return m_container;
    }

    double angleSign;

    private AutoCommandContainer(){}

    public static Command balanceCommand(DriveTrain driveTrain) {
        return new AutoDriveCommand(driveTrain, 1000, 0.75)
          .andThen(new AutoDriveCommand(driveTrain, -35000, 0.75))
          .andThen(new AutoDriveCommand(driveTrain, -40000, 0.35))
          .andThen(new InstantCommand(() -> { AutoCommandContainer.getInstance().angleSign = Math.signum(driveTrain.getPitch()); }))
          .andThen(new RepeatCommand(new InstantCommand(() -> { driveTrain.driveRaw(0, 0.25); }))
            .until(() -> {
              double angleSign = AutoCommandContainer.getInstance().angleSign;
              if(angleSign < 0) {
                return driveTrain.getPitch() > -8;
              } else if(angleSign > 0) {
                return driveTrain.getPitch() < 8;
              }
              return false;
              // return Math.signum(driveTrain.getPitch()) != AutoCommandContainer.getInstance().angleSign;
            }))
          .andThen(new AutoDriveCommand(driveTrain, 5500, 0.25))
          .andThen(new AutoBalance(driveTrain, 0.021, 0.0005, 0.025, 10, 0));
    }

    public static Command placeCubeMid(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
        return new InstantCommand(() -> {
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
          .andThen(new AutoDriveCommand(driveTrain, -5000, 0.3)); 
    }

    public static Command placeCubeHighCommand2(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
      return new InstantCommand(() -> {
        stinger.setGrabber(GrabberState.PINCH);
        pincher.setDropper(DropState.LOWERED);
        driveTrain.enableMotorBreak();
      })
      .andThen(new WaitCommand(0.6))

      // Raise shoulder
      .andThen(new InstantCommand(() -> { stinger.setShoulder(ShoulderState.RAISED); }))

      // Raise elbow until it can clear the dropper before raising it
      .andThen(new RepeatCommand(new ElbowToPosition(stinger, 78.5))
        .until(() -> { return stinger.m_elbowEncoder.getPosition() > 15; })

        // Raise dropper
        .andThen(new InstantCommand(() -> { pincher.setDropper(DropState.RAISED); }))

      // Raise elbow until it can clear the scoring shelf before dropping shoulder
      .andThen(new RepeatCommand(new ElbowToPosition(stinger, 78.5))
        .until(() -> { return stinger.m_elbowEncoder.getPosition() > 37.5; })

        // Drop shoulder
        .andThen(new InstantCommand(() -> { stinger.setShoulder(ShoulderState.LOWERED); }))

        // Extend to position
        .andThen(new ExtendToPosition(stinger, 380))

      // Raise elbow until it's in scoring position
      .andThen(new RepeatCommand(new ElbowToPosition(stinger, 78.5))
        .until(() -> { return stinger.m_elbowEncoder.getPosition() > 75; })

        // Wait for things to settle
        .andThen(new WaitCommand(0.4)))
      
      // Lower elbow until it's right above the scoring shelf
      .andThen(new RepeatCommand(new ElbowToPosition(stinger, 68))
        .until(() -> { return stinger.m_elbowEncoder.getPosition() < 70; })
      
      // Drop the cube
      .andThen(new InstantCommand(() -> { stinger.setGrabber(GrabberState.DROP); }))
      
      // Wait for the cube to settle
      .andThen(new WaitCommand(0.5))

      // Raise the elbow to clear the cube
      .andThen(new ElbowToPosition(stinger, 75))
      
      // All at the same time:
      // Retract the extension
      .andThen(new ExtendToPosition(stinger, 25))

      // Drive backwards 12500 at 40%
      //   halfway through: start going to travel position
      .andThen(new AutoDriveCommand(driveTrain, -8500, 0.5))
      .andThen(new DrivePosition(stinger, pincher))
      .andThen(new AutoDriveCommand(driveTrain, -6250, 0.4))
      .andThen(new WaitCommand(0.5))
      )));
    }

    public static Command placeCubeHighCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher){
        return new InstantCommand(() -> {
            stinger.setGrabber(GrabberState.PINCH);
            pincher.setDropper(DropState.LOWERED);
            driveTrain.enableMotorBreak();
          })
          .andThen(new WaitCommand(0.1))

          .andThen(new InstantCommand(() -> { stinger.setShoulder(ShoulderState.RAISED); }))

          .andThen(new WaitCommand(0.5))

          .andThen(new ElbowToPosition(stinger, 40))
          .andThen(new ExtendToPosition(stinger, 200))

          .andThen(new WaitCommand(0.65))

          .andThen(new InstantCommand(() -> { pincher.setDropper(DropState.RAISED); }))

          .andThen(new WaitCommand(0.4))
          
          .andThen(new ElbowToPosition(stinger, 78.5))
          .andThen(new ExtendToPosition(stinger, 335))
          .andThen(new InstantCommand(() -> { stinger.setShoulder(ShoulderState.LOWERED); }))
          .andThen(new WaitCommand(1.8))

          .andThen(new ElbowToPosition(stinger, 68))
          .andThen(new WaitCommand(0.2))
          .andThen(new InstantCommand(() -> { stinger.setGrabber(GrabberState.DROP); }))
          .andThen(new WaitCommand(0.2))
          .andThen(new ElbowToPosition(stinger, 75))
          .andThen(new ExtendToPosition(stinger, 50))
      
          .andThen(new WaitCommand(0.5))

          .andThen(new AutoDriveCommand(driveTrain, -12500, 0.4))
          .andThen(new WaitCommand(1))
          .andThen(new DrivePosition(stinger, pincher));
          // .andThen(new WaitCommand(0.25))
          // .andThen(new AutoDriveCommand(driveTrain, -5000, 0.3));
    }

    public static Command midThenCommunityCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher ) {
        return placeCubeMid(driveTrain,stinger,pincher)
          .andThen(new DrivePosition(stinger, pincher))
          .andThen(new AutoDriveCommand(driveTrain, -7500, 0.35))
          .andThen(new WaitCommand(0.5))
          .andThen(new InstantCommand(() -> { driveTrain.disableMotorBreak(); }))
          .andThen(new AutoDriveCommand(driveTrain, -154000, 0.35)); 
    }

    public static Command highThenCommunity(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
        return placeCubeHighCommand2(driveTrain, stinger, pincher)
        .andThen(new DrivePosition(stinger, pincher))
        .andThen(new AutoDriveCommand(driveTrain, -7500, 0.35))
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> { driveTrain.disableMotorBreak(); }))
        .andThen(new AutoDriveCommand(driveTrain, -154000, 0.35)); 
    }

    public static Command midThenBalanceCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher ) {
        return placeCubeMid(driveTrain, stinger, pincher)
        .andThen(new DrivePosition(stinger, pincher))
        .andThen(balanceCommand(driveTrain)); 
    }

    public static Command highThenBalanceCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher) {
        return placeCubeHighCommand(driveTrain, stinger, pincher)
        .andThen(new DrivePosition(stinger, pincher))
        .andThen(balanceCommand(driveTrain)); 
    }
    
}
