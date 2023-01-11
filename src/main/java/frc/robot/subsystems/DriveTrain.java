// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private final MotorControllerGroup m_controllerGroupL;
  private final MotorControllerGroup m_controllerGroupR;

  private final DifferentialDrive m_differentialDrive;

  public AHRS m_gyro;

  private final DifferentialDriveOdometry m_odometry; 

  private boolean m_turbo;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_motorFL = new WPI_TalonFX(Constants.kMotorFL);
    m_motorBL = new WPI_TalonFX(Constants.kMotorBL);
    m_motorFR = new WPI_TalonFX(Constants.kMotorFR);
    m_motorBR = new WPI_TalonFX(Constants.kMotorBR);

    // define closed loop for motors 
    m_motorFR.config_kD(0, Constants.DTConsts.kD, Constants.DTConsts.kTimeOut);
    m_motorFR.config_kF(0, Constants.DTConsts.kFF, Constants.DTConsts.kTimeOut); 
    m_motorFR.config_kI(0, Constants.DTConsts.kI, Constants.DTConsts.kTimeOut); 
    m_motorFR.config_kP(0, Constants.DTConsts.kP, Constants.DTConsts.kTimeOut); 

    m_motorBR.config_kD(0, Constants.DTConsts.kD, Constants.DTConsts.kTimeOut);
    m_motorBR.config_kF(0, Constants.DTConsts.kFF, Constants.DTConsts.kTimeOut); 
    m_motorBR.config_kI(0, Constants.DTConsts.kI, Constants.DTConsts.kTimeOut); 
    m_motorBR.config_kP(0, Constants.DTConsts.kP, Constants.DTConsts.kTimeOut); 
    
    m_motorFL.config_kD(0, Constants.DTConsts.kD, Constants.DTConsts.kTimeOut);
    m_motorFL.config_kF(0, Constants.DTConsts.kFF, Constants.DTConsts.kTimeOut); 
    m_motorFL.config_kI(0, Constants.DTConsts.kI, Constants.DTConsts.kTimeOut); 
    m_motorFL.config_kP(0, Constants.DTConsts.kP, Constants.DTConsts.kTimeOut); 

    m_motorBL.config_kD(0, Constants.DTConsts.kD, Constants.DTConsts.kTimeOut);
    m_motorBL.config_kF(0, Constants.DTConsts.kFF, Constants.DTConsts.kTimeOut); 
    m_motorBL.config_kI(0, Constants.DTConsts.kI, Constants.DTConsts.kTimeOut); 
    m_motorBL.config_kP(0, Constants.DTConsts.kP, Constants.DTConsts.kTimeOut); 

    m_motorFL.configOpenloopRamp(Constants.DTConsts.kRampRate);
    m_motorFR.configOpenloopRamp(Constants.DTConsts.kRampRate);
    m_motorBR.configOpenloopRamp(Constants.DTConsts.kRampRate);
    m_motorBL.configOpenloopRamp(Constants.DTConsts.kRampRate);
    
    m_motorFR.configAllowableClosedloopError(0, Constants.DTConsts.kClosedLoopError, Constants.DTConsts.kTimeOut); 
    m_motorFL.configAllowableClosedloopError(0, Constants.DTConsts.kClosedLoopError, Constants.DTConsts.kTimeOut); 
    m_motorBR.configAllowableClosedloopError(0, Constants.DTConsts.kClosedLoopError, Constants.DTConsts.kTimeOut); 
    m_motorBL.configAllowableClosedloopError(0, Constants.DTConsts.kClosedLoopError, Constants.DTConsts.kTimeOut); 

    m_motorFR.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.DTConsts.kStatusFrame, Constants.DTConsts.kTimeOut); 
    m_motorFL.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.DTConsts.kStatusFrame, Constants.DTConsts.kTimeOut); 
    m_motorBR.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.DTConsts.kStatusFrame, Constants.DTConsts.kTimeOut); 
    m_motorBL.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.DTConsts.kStatusFrame, Constants.DTConsts.kTimeOut); 

    m_motorBL.follow(m_motorFL);
    m_motorBR.follow(m_motorFR);

    

    m_controllerGroupL = new MotorControllerGroup(m_motorFL, m_motorBL);
    m_controllerGroupR = new MotorControllerGroup(m_motorFR, m_motorBR);

    m_differentialDrive = new DifferentialDrive(m_controllerGroupL, m_controllerGroupR);
    //m_differentialDrive.setDeadband(0.0);

    m_gyro = new AHRS(Port.kUSB);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0); 
    m_turbo = false;
  }

  public void enableMotorBreak()
  {
    m_motorFR.setNeutralMode(NeutralMode.Brake);
    m_motorFL.setNeutralMode(NeutralMode.Brake);
    m_motorBR.setNeutralMode(NeutralMode.Brake);
    m_motorBL.setNeutralMode(NeutralMode.Brake);
  }

  public void disableMotorBreak()
  {
    m_motorFR.setNeutralMode(NeutralMode.Coast);
    m_motorFL.setNeutralMode(NeutralMode.Coast);
    m_motorBR.setNeutralMode(NeutralMode.Coast);
    m_motorBL.setNeutralMode(NeutralMode.Coast);
  }

  public double getEncPos() {
    double encFL = m_motorFL.getSelectedSensorPosition(0);
    double encFR = -m_motorFR.getSelectedSensorPosition(0);
    double encBL = m_motorBL.getSelectedSensorPosition(0);
    double encBR = -m_motorBR.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Front Left Enc", encFL);
    SmartDashboard.putNumber("Front Right Enc", encFR);
    SmartDashboard.putNumber("Back Left Enc", encBL);
    SmartDashboard.putNumber("Back Right Enc", encBR);
    
    double position = (encFL + encFR + encBL + encBR) / 4;
    return position;
  }

public void resetHeading(){
  m_gyro.reset();
}

  public double getHeading() {
    return -Math.IEEEremainder(m_gyro.getAngle(), 360);
  }
  public double getTurnRate(){
    return m_gyro.getRate(); 
  }
  public void resetEncoders() {
    m_motorFL.setSelectedSensorPosition(0);
    m_motorFR.setSelectedSensorPosition(0);
    m_motorBL.setSelectedSensorPosition(0);
    m_motorBR.setSelectedSensorPosition(0);
  }

  public void drive(double xSpeed, double zRotation) {
    SmartDashboard.putNumber("Arcade Drive X Speed", xSpeed);
    SmartDashboard.putNumber("Arcade Drive Z Rotation", zRotation);
    if (!m_turbo) {
      m_differentialDrive.arcadeDrive(xSpeed * 0.80, zRotation * 0.80);
    } else {
      m_differentialDrive.arcadeDrive(xSpeed, zRotation);
    }
  }
  public void driveRaw(double zRotation, double xSpeed) {
    m_differentialDrive.arcadeDrive(zRotation,xSpeed, false);
  }

  public void tankDriveVolts(double left, double right)
  {
    this.m_motorFR.set(ControlMode.Current, right);
    this.m_motorFL.set(ControlMode.Current, left);
  }
  public void driveVelocity(double left, double right){
    double leftNativeVel = ConversionHelper.convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(left, Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution); 
    double rightNativeVel = ConversionHelper.convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(right, Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution); 
    this.m_motorFR.set(ControlMode.Velocity, rightNativeVel);
    this.m_motorFL.set(ControlMode.Velocity, leftNativeVel);
    SmartDashboard.putNumber("Left Target Vel", leftNativeVel);
    SmartDashboard.putNumber("Left Target Vs Actual", leftNativeVel-this.m_motorFL.getSelectedSensorVelocity()); 
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      ConversionHelper.convertTalonNativeToWPITrajectoryUnits(this.m_motorFL.getSelectedSensorVelocity(), Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution),
      ConversionHelper.convertTalonNativeToWPITrajectoryUnits(this.m_motorFR.getSelectedSensorVelocity(), Constants.DTConsts.kWheelDiameter, true, Constants.DTConsts.kTicksPerRevolution)
      );
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition( Rotation2d.fromDegrees(getHeading()),0.0,0, pose );
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters(); 
  }

  @Override
  public void periodic() {
    var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
    SmartDashboard.putNumber("current heading", getHeading());
    double leftDistance = ConversionHelper.convertTalonEncoderTicksToMeters((int)m_motorFL.getSelectedSensorPosition(), Constants.DTConsts.kWheelDiameter, Constants.DTConsts.kTicksPerRevolution, true); 
    double rightDistance = ConversionHelper.convertTalonEncoderTicksToMeters((int)m_motorFR.getSelectedSensorPosition(), Constants.DTConsts.kWheelDiameter, Constants.DTConsts.kTicksPerRevolution, true);
    getEncPos();

    // Update the pose
    m_odometry.update(gyroAngle, leftDistance, rightDistance);
  }

  public void enableTurbo() {
    m_turbo = true;
  }

  public void disableTurbo() {
    m_turbo = false;
  }
}
