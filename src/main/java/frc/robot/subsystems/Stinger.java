// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stinger extends SubsystemBase {
  private Solenoid m_stingerSolenoid;
  private Solenoid m_grabberSolenoid;
  private CANSparkMax m_extendSpark;
  private CANSparkMax m_raiseSpark;
  private ShoulderState m_shoulderState;
  private GrabberState m_grabberState;

  private double kExtendP = 0.1;
  private double kExtendI = 1e-4;
  private double kExtendD = 1;
  private double kExtendIZ = 0;
  private double kExtendFF = 0;
  private double kExtendMin = -.25;
  private double kExtendMax = .25;
  private SparkMaxPIDController m_extendController;
  private RelativeEncoder m_extendEncoder;

  public enum ElbowDirection { RAISE, LOWER, STOP };
  public enum StingerDirection { EXTEND, RETRACT, STOP };
  public enum ShoulderState { RAISED, LOWERED };
  public enum GrabberState { PINCH, DROP };
 
  /** Creates a new Stinger. */
  public Stinger() {
    m_stingerSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kStingerSolenoid);
    m_grabberSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kGrabbySolenoidIndex);

    

    m_extendSpark = new CANSparkMax(Constants.kExtendStingerSpark, MotorType.kBrushless);
    m_raiseSpark = new CANSparkMax(Constants.kRaiseStingerSpark, MotorType.kBrushless);

    m_extendSpark.setIdleMode(IdleMode.kBrake);
    m_raiseSpark.setIdleMode(IdleMode.kBrake);

    m_extendController = m_extendSpark.getPIDController();
    m_extendEncoder = m_extendSpark.getEncoder();
    m_extendController.setP(kExtendP);
    m_extendController.setI(kExtendI);
    m_extendController.setD(kExtendD);
    m_extendController.setIZone(kExtendIZ);
    m_extendController.setFF(kExtendFF);
    m_extendController.setOutputRange(kExtendMin, kExtendMax);

    m_shoulderState = m_stingerSolenoid.get() ? ShoulderState.RAISED : ShoulderState.LOWERED;

    m_grabberState = m_grabberSolenoid.get() ? GrabberState.PINCH : GrabberState.DROP;
    m_extendSpark.getEncoder().setPosition(0.0);
    m_raiseSpark.getEncoder().setPosition(0.0);
    SmartDashboard.putNumber("Elbow Speed", 0.25);
    SmartDashboard.putNumber("Extend Speed", 0.25);
    SmartDashboard.putNumber("Pickup elbow position", 16);
    SmartDashboard.putNumber("Pickup elbow p", kExtendP);
    SmartDashboard.putNumber("Pickup elbow i", kExtendI);
    SmartDashboard.putNumber("Pickup elbow d", kExtendD);
    SmartDashboard.putNumber("Pickup elbow iz", kExtendIZ);
    SmartDashboard.putNumber("Pickup elbow ff", kExtendFF);
    SmartDashboard.putNumber("Pickup elbow min", kExtendMin);
    SmartDashboard.putNumber("Pickup elbow max", kExtendMax);
    SmartDashboard.putNumber("Pickup elbow set point", 0);
    SmartDashboard.putNumber("Pickup extend position", 181);
  }

  private double elbowSpeed = 0.0;
  private double extendSpeed = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowSpeed = SmartDashboard.getNumber("Elbow Speed", 0.25);
    extendSpeed = SmartDashboard.getNumber("Extend Speed", 0.25);
    
    double p = SmartDashboard.getNumber("Pickup elbow p", 0);
    double i = SmartDashboard.getNumber("Pickup elbow i", 0);
    double d = SmartDashboard.getNumber("Pickup elbow d", 0);
    double iz = SmartDashboard.getNumber("Pickup elbow iz", 0);
    double ff = SmartDashboard.getNumber("Pickup elbow ff", 0);
    if (p != kExtendP) {
      m_extendController.setP(p);
      kExtendP = p;
    }
    if (i != kExtendI) {
      m_extendController.setI(i);
      kExtendD = i;
    }
    if (d != kExtendD) {
      m_extendController.setD(d);
      kExtendD = d;
    }
    if (iz != kExtendIZ) {
      m_extendController.setI(iz);
      kExtendIZ = iz;
    }
    if (ff != kExtendFF) {
      m_extendController.setI(ff);
      kExtendFF = ff;
    }

    
    SmartDashboard.putNumber("Elbow Position", m_raiseSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Extend Position", m_extendSpark.getEncoder().getPosition());
  }

  public void enableClosedLoop() {
    double setPoint = SmartDashboard.getNumber("Pickup elbow set point", 0);
    m_extendController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }
  
  
  public void setGrabber(GrabberState state) {
    m_grabberState = state;
    m_grabberSolenoid.set(state == GrabberState.PINCH);
  }

  public void toggleGrabber() {
    setGrabber(m_grabberState == GrabberState.DROP ? GrabberState.PINCH : GrabberState.DROP);
  }

  public void setElbow(ElbowDirection direction) {
    m_raiseSpark.set(direction == ElbowDirection.STOP ? 0
      : direction == ElbowDirection.LOWER ? -elbowSpeed : elbowSpeed);
  }

  public void setStinger(StingerDirection direction) {
    m_extendSpark.set(direction == StingerDirection.STOP ? 0
      : (direction == StingerDirection.EXTEND ? extendSpeed : -extendSpeed));
  }

  public void setShoulder(ShoulderState state) {
    m_shoulderState = state;
    m_stingerSolenoid.set(state == ShoulderState.RAISED);
  }

  public void toggleShoulder() {
    setShoulder(m_shoulderState == ShoulderState.RAISED ? ShoulderState.LOWERED : ShoulderState.RAISED);
  }
}
