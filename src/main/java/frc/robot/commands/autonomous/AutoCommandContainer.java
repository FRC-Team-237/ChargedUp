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

    public static Command balanceCommand(DriveTrain driveTrain)
    {
        return new AutoDriveCommand(driveTrain, -35000, 0.75)
        // .andThen(new AutoDriveCommand(driveTrain, -35000, 0.75))
        .andThen(new AutoDriveCommand(driveTrain, -50000, 0.35))
        .andThen(new RepeatCommand(new InstantCommand(() -> { driveTrain.driveRaw(0, -0.25); }))
          .until(() -> { return driveTrain.getPitch() > 0; }))
        // .andThen(new AutoDriveCommand(driveTrain, 4500, 0.25))
        .andThen(new AutoBalance(driveTrain, 0.021, 0.0005, 0.025, 10, 0));
    }
    public static Command placeCubeMid(DriveTrain driveTrain, Stinger stinger, Pincher pincher)
    {
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
    public static Command placeCubeHighCommand(DriveTrain driveTrain, Stinger stinger, Pincher pincher){
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
      
          .andThen(new ElbowToPosition(stinger, 78.5))
          .andThen(new ExtendToPosition(stinger, 335))
          .andThen(new WaitCommand(0.2))
          .andThen(new InstantCommand(() -> { pincher.setDropper(DropState.RAISED); }))
      
          .andThen(new WaitCommand(1.2))
          .andThen(new ElbowToPosition(stinger, 68))
          .andThen(new WaitCommand(0.25))
          .andThen(new InstantCommand(() -> { stinger.setGrabber(GrabberState.DROP); }))
      
          .andThen(new WaitCommand(0.25))
          .andThen(new AutoDriveCommand(driveTrain, -5000, 0.3)); 
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
        return placeCubeHighCommand(driveTrain, stinger, pincher)
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
