// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoCommandGroup1;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoGrabbyCommand;


import frc.robot.commands.GoalFinder;
import frc.robot.commands.LongBall;
import frc.robot.commands.TargetFinder;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;

// import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.TalonRamseteControllerAbstraction;
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

  private AutoCommandGroup1 m_autoCommandFarLeft;
  private AutoCommandGroup1 m_autoCommandCloseLeft;
  private AutoCommandGroup1 m_autoCommandCloseRight;
  private AutoCommandGroup1 m_autoCommandCenter;

  private XboxController m_xboxController;
  public static Joystick m_flightStick = new Joystick(2);
  public static JoystickButton m_grabButton = new JoystickButton(m_flightStick, 1);
  public static JoystickButton m_wheelsButton = new JoystickButton(m_flightStick, 6);
  public static Joystick panel = new Joystick(1);
  public static JoystickButton m_autoSwitchOne = new JoystickButton(panel, Constants.kAutoSwitchOne);
  public static JoystickButton m_autoSwitchTwo = new JoystickButton(panel, Constants.kAutoSwitchTwo);
  public static JoystickButton m_autoSwitchThree = new JoystickButton(panel, Constants.kAutoSwitchThree);
  public static JoystickButton m_autoSwitchFour = new JoystickButton(panel, Constants.kAautoSwitchFour);
  public static JoystickButton m_shoot = new JoystickButton(panel, Constants.kShoot);
  public static JoystickButton m_manualShoot = new JoystickButton(panel, Constants.kManualShoot);
  public static JoystickButton m_killSwitch = new JoystickButton(panel, Constants.kKillSwitch);
  public static JoystickButton m_hangOneUp = new JoystickButton(panel, Constants.kHangOneUp);
  public static JoystickButton m_hangOneDown = new JoystickButton(panel, Constants.kHangOnedown);
  public static JoystickButton m_hangTwoUp = new JoystickButton(panel, Constants.kHangTwoUp);
  public static JoystickButton m_hangTwoDown = new JoystickButton(panel, Constants.kHangTwoDown);
  public static JoystickButton m_autoPickup = new JoystickButton(panel, Constants.kAutoPickup);
  public static JoystickButton m_eject = new JoystickButton(panel, Constants.kEject);

  private DriveTrain m_driveTrain;
  // public Grabber m_grabber;
  public AutoGrabbyCommand m_grabCommand;
  private Intake m_intake;
  private Stinger m_stinger;


  private boolean m_isRedAlliance;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 
    m_xboxController = new XboxController(0);
    m_flightStick.setXChannel(0);
    m_flightStick.setYChannel(1);
    m_flightStick.setZChannel(2);
    m_driveTrain = new DriveTrain();
    // m_grabber = new Grabber();
    // m_grabCommand = new AutoGrabbyCommand(m_grabber);
    m_intake = new Intake();
    m_stinger = new Stinger();
   

    
  
  
    m_isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;

    // m_autoCommandFarLeft = new AutoCommandGroup1(m_grabber, m_driveTrain, 170, -25000);
    // m_autoCommandCloseLeft = new AutoCommandGroup1(m_grabber, m_driveTrain, 165, -40000);
    // m_autoCommandCloseRight = new AutoCommandGroup1(m_grabber, m_driveTrain, -170, -15000);
    // m_autoCommandCenter = new AutoCommandGroup1(m_grabber, m_shooter, m_driveTrain, 180, -50000); // Not done

    Leds.getInstance().setIsRedAlliance(m_isRedAlliance);
    Leds.getInstance().resetColor();
    

    // Configure the button bindings
    configureButtonBindings();
    if (m_isRedAlliance) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    }

    // m_driveTrain.setDefaultCommand(
    //   new RunCommand(
    //     () -> 
    //       m_driveTrain.drive(
    //         m_xboxController.getLeftX(),
    //         m_xboxController.getLeftY()
    //     ), m_driveTrain
    //   )
    // );
    m_driveTrain.setDefaultCommand(
      new RunCommand(
        () -> {
          m_driveTrain.drive(
            m_flightStick.getX(),
            m_flightStick.getY()
          );
          m_driveTrain.setScale(
            ConversionHelper.mapRange(-m_flightStick.getZ(), -1, 1, .2, 1)
          );
        }
        , m_driveTrain
      )
    );
  }
  public void resetOdometry()
  {
    m_driveTrain.resetHeading();
    m_driveTrain.resetEncoders();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(){
    driversControlPlan();
  }
  private void driversControlPlan(){
    new JoystickButton(m_flightStick, 8)
    .whileTrue(
      new InstantCommand(
        () -> {
          m_stinger.toggleStinger();
        },
        m_stinger
      )
    );
    new JoystickButton(m_flightStick, 9)
    .whileTrue(
      new InstantCommand(
        () -> {
          m_intake.raisevlow();
        },
        m_intake
      )
    );

    new JoystickButton(m_flightStick, 1)
    .whileTrue(
      new RepeatCommand(new InstantCommand(
        () -> {
          m_intake.closeIntake();
        },
        m_intake
      )
    ))
    .whileFalse(
      new InstantCommand(
        () -> {
          m_intake.openIntake();
        },
        m_intake
      )
    );
    
    new JoystickButton(m_flightStick, 6)
      .whileTrue(new RepeatCommand(new InstantCommand(
        () -> {
          m_stinger.lowerElbow();
        },
        m_stinger
      )))
      .whileFalse(new InstantCommand(
        () -> {
          m_stinger.stopElbow();
        }
      )
    );
    
    new JoystickButton(m_flightStick, 7)
      .whileTrue(new RepeatCommand(new InstantCommand(
        () -> {
          m_stinger.raiseElbow();
        },
        m_stinger
      )))
      .whileFalse(new InstantCommand(
        () -> {
          m_stinger.stopElbow();
        }
      )
    );

    new JoystickButton(m_flightStick, 10)
      .whileTrue(new RepeatCommand(new InstantCommand(
        () -> {
          m_stinger.retractStinger();
        },
        m_stinger
      )))
      .whileFalse(new InstantCommand(
        () -> {
          m_stinger.stopStinger();
        }
      )
    );
    
    new JoystickButton(m_flightStick, 11)
      .whileTrue(new RepeatCommand(new InstantCommand(
        () -> {
          m_stinger.extendStinger();
        },
        m_stinger
      )))
      .whileFalse(new InstantCommand(
        () -> {
          m_stinger.stopStinger();
        }
      )
    );
    
    new JoystickButton(m_xboxController, Button.kX.value)
      .whileTrue(
        new TargetFinder(m_driveTrain, Constants.LimeLight.kGoalDriveP)
        .andThen(new InstantCommand(m_intake::closeIntake, m_intake))
      );
      new JoystickButton(m_xboxController, Button.kRightBumper.value)
      .whileTrue(
        new RepeatCommand(new InstantCommand(
          () -> {
            m_intake.closeIntake();
          },
          m_intake
        )
      ))
      .whileFalse(
        new InstantCommand(
          () -> {
            m_intake.openIntake();
          },
          m_intake
        )
      );

    new JoystickButton(m_xboxController, Button.kBack.value)
      .whileTrue(
        new InstantCommand(m_driveTrain::resetHeading).andThen(new InstantCommand(m_driveTrain::resetEncoders, m_driveTrain))
      // .andThen(new TurnToAngle(180, m_driveTrain, 0.0113, 0.0000, 0.0025, 0))
        .andThen(new TurnToAngle(
          180,
          m_driveTrain,
          SmartDashboard.getNumber("TurnToAngle P", 0.0113),
          SmartDashboard.getNumber("TurnToAngle I", 0.0000),
          SmartDashboard.getNumber("TurnToAngle D", 0.0025),
          SmartDashboard.getNumber("TurnToAngle Deadband", 0)
        ))
    );

    new JoystickButton(m_xboxController, Button.kStart.value)
      .whileTrue(new AutoDriveCommand(m_driveTrain, 180000, 0.5)
    );


    // control pannel buttons 

    new JoystickButton(panel, Constants.kAutoPickup)
      .whileTrue(m_grabCommand);
    
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
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
     return new InstantCommand();
  }
}


