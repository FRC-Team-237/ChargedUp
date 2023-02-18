// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ConversionHelper;
import frc.robot.commands.AutoDriveCommand;

import frc.robot.commands.TargetFinder;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.TalonRamseteControllerAbstraction;
import frc.robot.subsystems.Pincher.DropState;
import frc.robot.subsystems.Pincher.PinchState;
import frc.robot.subsystems.Stinger.ElbowDirection;
import frc.robot.subsystems.Stinger.GrabberState;
import frc.robot.subsystems.Stinger.StingerDirection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private XboxController m_xboxController;
  public static Joystick m_flightStick = new Joystick(2);
  public static Joystick panel = new Joystick(1);
  public static Joystick m_arcadePanel = new Joystick(0);

  private DriveTrain m_driveTrain;
  private Pincher m_pincher;
  private Stinger m_stinger;

  private boolean m_isRedAlliance;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_xboxController = new XboxController(0);
    m_flightStick.setXChannel(0);
    m_flightStick.setYChannel(1);
    m_flightStick.setZChannel(2);
    m_driveTrain = new DriveTrain();
    m_pincher = new Pincher();
    m_stinger = new Stinger();
  
    m_isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;

    // Leds.getInstance().setIsRedAlliance(m_isRedAlliance);
    // Leds.getInstance().resetColor();

    // Configure the button bindings
    configureButtonBindings();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(m_isRedAlliance ? 0 : 1);

    // m_stinger.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       int povIndex = m_arcadePanel.getPOV();
    //       if(povIndex == 0) {
    //         m_stinger.setStinger(StingerDirection.EXTEND);
    //       } else if(povIndex == 180) {
    //         m_stinger.setStinger(StingerDirection.RETRACT);
    //       } else {
    //         m_stinger.setStinger(StingerDirection.STOP);
    //       }
    //     }, m_stinger));

    m_driveTrain.setDefaultCommand(
      new RunCommand(
        () -> {
          m_driveTrain.drive(
            m_flightStick.getX(),
            m_flightStick.getY());
          m_driveTrain.setScale(
            ConversionHelper.mapRange(-m_flightStick.getZ(), -1, 1, .2, 1));
        }, m_driveTrain));
  }

  public void resetOdometry() {
    m_driveTrain.resetGyro();
    m_driveTrain.resetEncoders();
  }
  
  enum Input {
    TOGGLE_SHOULDER,
    TOGGLE_DROPPER,
    RAISE_DROPPER,
    LOWER_DROPPER,
    PINCH,
    LOWER_ELBOW,
    RAISE_ELBOW,
    RETRACT_STINGER,
    EXTEND_STINGER,
    GRAB
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    class ControlMap {
      public GenericHID controller;
      public String name;

      public ControlMap(GenericHID controller, String name) {
        this.controller = controller;
        this.name = name;
      }
    }
    class InputButton {
      public String description;
      public JoystickButton button;
      public ControlMap controller;
      public int buttonIndex;

      public InputButton(ControlMap controller, String description, int buttonIndex) {
        this.controller = controller;
        this.buttonIndex = buttonIndex;
        this.description = description;
        this.button = new JoystickButton(this.controller.controller, this.buttonIndex);
      }
    }

    ControlMap flightStick = new ControlMap(m_flightStick, "Driver Stick");
    ControlMap arcadePanel = new ControlMap(m_arcadePanel, "Panel");
    
    HashMap<Input, InputButton> keyMap = new HashMap<Input, InputButton>();

    // keyMap.put(Input.TOGGLE_DROPPER,  new InputButton(flightStick, "Toggle Dropper",  9));
    // keyMap.put(Input.PINCH,           new InputButton(flightStick, "Pinch",           1));
    // keyMap.put(Input.TOGGLE_SHOULDER, new InputButton(flightStick, "Toggle Shoulder", 8));
    // keyMap.put(Input.LOWER_ELBOW,     new InputButton(flightStick, "Lower Elbow",     6));
    // keyMap.put(Input.RAISE_ELBOW,     new InputButton(flightStick, "Raise Elbow",     7));
    // keyMap.put(Input.RETRACT_STINGER, new InputButton(flightStick, "Retract Stinger", 10));
    // keyMap.put(Input.EXTEND_STINGER,  new InputButton(flightStick, "Extend Stinger",  11));
    // keyMap.put(Input.GRAB,            new InputButton(flightStick, "Grab",            2));

    // keyMap.put(Input.TOGGLE_DROPPER,  new InputButton(arcadePanel, "Toggle Dropper",  3));
    keyMap.put(Input.PINCH,           new InputButton(flightStick, "Pinch",           1));
    keyMap.put(Input.LOWER_DROPPER,   new InputButton(flightStick, "Lower Dropper",   6));
    keyMap.put(Input.RAISE_DROPPER,   new InputButton(flightStick, "Raise Dropper",   7));
    keyMap.put(Input.TOGGLE_SHOULDER, new InputButton(arcadePanel, "Toggle Shoulder", 6));
    keyMap.put(Input.LOWER_ELBOW,     new InputButton(arcadePanel, "Lower Elbow",     2));
    keyMap.put(Input.RAISE_ELBOW,     new InputButton(arcadePanel, "Raise Elbow",     4));
    keyMap.put(Input.RETRACT_STINGER, new InputButton(arcadePanel, "Retract Stinger", 1));
    keyMap.put(Input.EXTEND_STINGER,  new InputButton(arcadePanel, "Extend Stinger",  3));
    // keyMap.put(Input.RETRACT_STINGER, new InputButton(arcadePanel, "Retract Stinger", 10));
    // keyMap.put(Input.EXTEND_STINGER,  new InputButton(arcadePanel, "Extend Stinger",  11));
    keyMap.put(Input.GRAB,            new InputButton(arcadePanel, "Grab",            5));

    keyMap.forEach((input, value) -> {
      SmartDashboard.putString(value.description, value.controller.name + " [" + value.buttonIndex + "]");
    });

    keyMap.get(Input.GRAB).button
    .whileTrue(new RepeatCommand(new InstantCommand(() -> {
      m_stinger.setGrabber(GrabberState.PINCH);
    }, m_stinger)))
    .whileFalse(new InstantCommand(() -> {
      m_stinger.setGrabber(GrabberState.DROP);
    }, m_stinger));

    keyMap.get(Input.TOGGLE_SHOULDER).button
      .whileTrue(new InstantCommand(() -> {
        m_stinger.toggleShoulder();
      }, m_stinger));

    // keyMap.get(Input.TOGGLE_DROPPER).button
    //   .whileTrue(new InstantCommand(() -> {
    //     m_pincher.toggleDropper();
    //   }, m_pincher));
    keyMap.get(Input.LOWER_DROPPER).button
      .whileTrue(new InstantCommand(() -> {
        m_pincher.setDropper(DropState.LOWERED);
      }));
    keyMap.get(Input.RAISE_DROPPER).button
      .whileTrue(new InstantCommand(() -> {
        m_pincher.setDropper(DropState.RAISED);
      }));


    keyMap.get(Input.PINCH).button
      .whileTrue(new RepeatCommand(new InstantCommand(() -> {
        m_pincher.setPincher(PinchState.CLOSED);
      }, m_pincher)))
      .whileFalse(new InstantCommand(() -> {
        m_pincher.setPincher(PinchState.OPEN);
      }, m_pincher));
    
    keyMap.get(Input.LOWER_ELBOW).button
      .whileTrue(new RepeatCommand(new InstantCommand(() -> {
          m_stinger.setElbow(ElbowDirection.LOWER);
        }, m_stinger)))
      .whileFalse(new InstantCommand(() -> {
          m_stinger.setElbow(ElbowDirection.STOP);
        }));

    keyMap.get(Input.RAISE_ELBOW).button
      .whileTrue(new RepeatCommand(new InstantCommand(() -> {
          m_stinger.setElbow(ElbowDirection.RAISE);
        }, m_stinger)))
      .whileFalse(new InstantCommand(() -> {
          m_stinger.setElbow(ElbowDirection.STOP);
        }));

    keyMap.get(Input.RETRACT_STINGER).button
      .whileTrue(new RepeatCommand(new InstantCommand(() -> {
          m_stinger.setStinger(StingerDirection.RETRACT);
        }, m_stinger)))
      .whileFalse(new InstantCommand(() -> {
          m_stinger.setStinger(StingerDirection.STOP);
        }));

    keyMap.get(Input.EXTEND_STINGER).button
      .whileTrue(new RepeatCommand(new InstantCommand(() -> {
          m_stinger.setStinger(StingerDirection.EXTEND);
        }, m_stinger)))
      .whileFalse(new InstantCommand(() -> {
          m_stinger.setStinger(StingerDirection.STOP);
        }));

    // new JoystickButton(m_xboxController, Button.kX.value)
    //   .whileTrue(
    //     new TargetFinder(m_driveTrain, Constants.LimeLight.kGoalDriveP)
    //       .andThen(new InstantCommand(() -> {
    //         m_pincher.setPincher(PinchState.CLOSED);
    //       } , m_pincher)));

    // new JoystickButton(m_xboxController, Button.kRightBumper.value)
    //   .whileTrue(new RepeatCommand(new InstantCommand(() -> {
    //         m_pincher.setPincher(PinchState.CLOSED);
    //       }, m_pincher)))
    //   .whileFalse(new InstantCommand(() -> {
    //         m_pincher.setPincher(PinchState.CLOSED);
    //       }, m_pincher));

    // new JoystickButton(m_xboxController, Button.kBack.value)
    //   .whileTrue(
    //     new InstantCommand(m_driveTrain::resetGyro)
    //       .andThen(new InstantCommand(m_driveTrain::resetEncoders, m_driveTrain))
    //       .andThen(new TurnToAngle(
    //         180,
    //         m_driveTrain,
    //         SmartDashboard.getNumber("TurnToAngle P", 0.0113),
    //         SmartDashboard.getNumber("TurnToAngle I", 0.0000),
    //         SmartDashboard.getNumber("TurnToAngle D", 0.0025),
    //         SmartDashboard.getNumber("TurnToAngle Deadband", 0)
    //       ))
    // );

    // new JoystickButton(m_xboxController, Button.kStart.value)
    //   .whileTrue(new AutoDriveCommand(m_driveTrain, 180000, 0.5));

    // control panel buttons 

    // new JoystickButton(panel, Constants.kAutoPickup)
    //   .whileTrue(m_grabCommand);
    
    // new JoystickButton(m_flightStick, 11)
    //   .whileTrue(new AutoBalance(
    //     m_driveTrain,
    //     SmartDashboard.getNumber("AutoBalance P", 0.0113),
    //     SmartDashboard.getNumber("AutoBalance I", 0.0000),
    //     SmartDashboard.getNumber("AutoBalance D", 0.0025),
    //     SmartDashboard.getNumber("AutoBalance Deadband", 0)
    //   ));
  }

  public Command createAutoNavigationCommand(Pose2d start, List<Translation2d> waypoints, Pose2d end) {
    System.out.println("Creating Auto Command");
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DTConsts.KS, Constants.DTConsts.KV, Constants.DTConsts.KA),
        Constants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.DTConsts.MAX_VELOCITY, Constants.DTConsts.MAX_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    System.out.println("Generated Trajectory");
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_driveTrain::getPose,
        new TalonRamseteControllerAbstraction(Constants.DTConsts.RAMSETE_B, Constants.DTConsts.RAMSETE_ZETA),

        Constants.kDriveKinematics,
       
        m_driveTrain::driveVelocity, m_driveTrain);

    // Run path following command, then stop at the end.
    System.out.println("Finished Creating Auto Command");
    return ramseteCommand.andThen(() -> m_driveTrain.drive(0, 0));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   // return m_autoCommand;
  // }
}


