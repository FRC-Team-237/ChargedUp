// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ConversionHelper;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX m_motorFR;
  private final WPI_TalonFX m_motorFL;
  private final WPI_TalonFX m_motorBR;
  private final WPI_TalonFX m_motorBL;
  private List<WPI_TalonFX> m_motors;

  private final MotorControllerGroup m_controllerGroupL;
  private final MotorControllerGroup m_controllerGroupR;

  private final DifferentialDrive m_differentialDrive;
  private final DifferentialDriveOdometry m_odometry;

  public AHRS m_gyro;

  private double m_speedScale;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_motorFL = new WPI_TalonFX(Constants.kMotorFL);
    m_motorBL = new WPI_TalonFX(Constants.kMotorBL);
    m_motorFR = new WPI_TalonFX(Constants.kMotorFR);
    m_motorBR = new WPI_TalonFX(Constants.kMotorBR);

    m_motors = List.of(m_motorFL, m_motorFR, m_motorBR, m_motorBL);

    // define closed loop for motors 
    m_motors.forEach(motor -> {
      motor.config_kD(0, Constants.DTConsts.kD, Constants.DTConsts.kTimeOut);
      motor.config_kF(0, Constants.DTConsts.kFF, Constants.DTConsts.kTimeOut); 
      motor.config_kI(0, Constants.DTConsts.kI, Constants.DTConsts.kTimeOut); 
      motor.config_kP(0, Constants.DTConsts.kP, Constants.DTConsts.kTimeOut);
      motor.configOpenloopRamp(Constants.DTConsts.kRampRate);
      motor.configAllowableClosedloopError(0, Constants.DTConsts.kClosedLoopError, Constants.DTConsts.kTimeOut);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.DTConsts.kStatusFrame, Constants.DTConsts.kTimeOut);
    });

    m_motorBL.follow(m_motorFL);
    m_motorBR.follow(m_motorFR);

    m_controllerGroupL = new MotorControllerGroup(m_motorFL, m_motorBL);
    m_controllerGroupR = new MotorControllerGroup(m_motorFR, m_motorBR);

    m_differentialDrive = new DifferentialDrive(m_controllerGroupL, m_controllerGroupR);
    m_differentialDrive.setDeadband(0.05);

    m_gyro = new AHRS(Port.kMXP);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(0.0,0.0),
      0, 0); 
    m_speedScale = 1.0;
  }

  public void enableMotorBreak() {
    m_motors.forEach(motor -> {
      motor.setNeutralMode(NeutralMode.Brake);
    });
  }

  public void disableMotorBreak() {
    m_motors.forEach(motor -> {
      motor.setNeutralMode(NeutralMode.Coast);
    });
  }

  public double getEncPos() {
    double encFL =  m_motorFL.getSelectedSensorPosition(0);
    double encFR = -m_motorFR.getSelectedSensorPosition(0);
    double encBL =  m_motorBL.getSelectedSensorPosition(0);
    double encBR = -m_motorBR.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Front Left Enc", encFL);
    SmartDashboard.putNumber("Front Right Enc", encFR);
    SmartDashboard.putNumber("Back Left Enc", encBL);
    SmartDashboard.putNumber("Back Right Enc", encBR);

    double position = (encFL + encFR + encBL + encBR) / 4;
    return position;
  }

  public double getHeading() {
    return -Math.IEEEremainder(m_gyro.getAngle(), 360);
  }
  public void resetGyro() {
    m_gyro.reset();
  }

  public double getPitch() {
    return Math.IEEEremainder(m_gyro.getPitch(), 360);
  }

  public double getTurnRate() {
    return 0.0; //m_gyro.getRate(); 
  }

  public void resetEncoders() {
    m_motors.forEach(motor -> {
      motor.setSelectedSensorPosition(0);
    });
  }

  public void drive(double xSpeed, double zRotation) {
    SmartDashboard.putNumber("Arcade Drive X Speed", xSpeed);
    SmartDashboard.putNumber("Arcade Drive Z Rotation", zRotation);

    m_differentialDrive.curvatureDrive(
      xSpeed * m_speedScale,
      zRotation  * m_speedScale,
      true);
  }

  public void driveRaw(double zRotation, double xSpeed) {
    m_differentialDrive.arcadeDrive(zRotation,xSpeed, false);
  }

  public void tankDriveVolts(double left, double right) {
    this.m_motorFR.set(ControlMode.Current, right);
    this.m_motorFL.set(ControlMode.Current, left);
  }

  public void driveVelocity(double left, double right) {
    double leftNativeVel = ConversionHelper.trajectoryToSRXUnits(left, Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution); 
    double rightNativeVel = ConversionHelper.trajectoryToSRXUnits(right, Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution); 

    this.m_motorFR.set(ControlMode.Velocity, rightNativeVel);
    this.m_motorFL.set(ControlMode.Velocity, leftNativeVel);
    m_differentialDrive.feed();
    SmartDashboard.putNumber("Left Target Vel", leftNativeVel);
    SmartDashboard.putNumber("Left Target Vs Actual", leftNativeVel - this.m_motorFL.getSelectedSensorVelocity()); 
  }

  public void arcadeDriveVelocity(double xSpeed, double zRotation) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    double leftVel = ConversionHelper.mapRange(speeds.left, -1.0, 1.0, ConversionHelper.toMeters(-13.5), ConversionHelper.toMeters(13.5));
    double rightVel = ConversionHelper.mapRange(speeds.right, -1.0, 1.0, ConversionHelper.toMeters(-13.5), ConversionHelper.toMeters(13.5));
    driveVelocity(leftVel, rightVel);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        ConversionHelper.SRXToTrajectoryUnits(this.m_motorFL.getSelectedSensorVelocity(), Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution),
        ConversionHelper.SRXToTrajectoryUnits(this.m_motorFR.getSelectedSensorVelocity(), Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), 0.0, 0, pose);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters(); 
  }

  @Override
  public void periodic() {
    //var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
    SmartDashboard.putNumber("Heading", getHeading());
    double leftDistance = ConversionHelper.convertTalonEncoderTicksToMeters((int) m_motorFL.getSelectedSensorPosition(), Constants.DTConsts.kWheelDiameter, Constants.DTConsts.kTicksPerRevolution, true); 
    double rightDistance = ConversionHelper.convertTalonEncoderTicksToMeters((int) m_motorFR.getSelectedSensorPosition(), Constants.DTConsts.kWheelDiameter, Constants.DTConsts.kTicksPerRevolution, true);
    getEncPos();
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Drive Speed", m_speedScale);

    // Update the pose
    //m_odometry.update(gyroAngle, leftDistance, rightDistance);
  }

  public void setScale(double Scale) {
    m_speedScale = Scale;
  }
}