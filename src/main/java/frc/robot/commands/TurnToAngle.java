// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double targetAngle,DriveTrain vroomVroom,double p,double i,double d,double deadband) {
    super(
        // The controller that the command will use
        new PIDController(p, i, d),
        // This should return the measurement
        vroomVroom::getHeading,
        targetAngle,
        // This should return the setpoint (can also be a constant)
      
        // This uses the output
        output -> {vroomVroom.driveRaw(-output, 0);
          // Use the output here
        });
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(deadband, 4);
      
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
