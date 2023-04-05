// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stinger extends SubsystemBase {
  private Solenoid m_stingerSolenoid;
  private Solenoid m_grabberSolenoid;
  private CANSparkMax m_extendSpark;
  private CANSparkMax m_elbowSpark;
  private ShoulderState m_shoulderState;
  public GrabberState m_grabberState;

  public DigitalInput m_shoulderInPlace;

  private double kExtendP = 0.8;
  private double kExtendI = 1e-4;
  private double kExtendD = 1;
  private double kExtendIZ = 0;
  private double kExtendFF = 0;
  private double kExtendMin = -.6;
  private double kExtendMax = .6;

  private double kElbowP = 0.1;
  private double kElbowI = 1e-4;
  private double kElbowD = 1;
  private double kElbowIZ = 0;
  private double kElbowFF = 1e-5;
  private double kElbowMin = -.25;
  private double kElbowMax = .25;

  private final double kExtendDeadband = 2.0;
  private final double kElbowDeadband = 0.05;

  private SparkMaxPIDController m_extendController;
  public RelativeEncoder m_extendEncoder;
  private SparkMaxPIDController m_elbowController;
  public RelativeEncoder m_elbowEncoder;

  public double kExtendSetpoint;
  public double kElbowSetpoint;

  private ElbowDirection elbowDirection;
  private StingerDirection stingerDirection;

  private ShuffleboardTab tab = Shuffleboard.getTab("PID Testing");
  private GenericEntry extendPEntry = tab.add("Pickup Extend P", 0.8).getEntry();
  private GenericEntry extendIEntry = tab.add("Pickup Extend I", 1e-4).getEntry();
  private GenericEntry extendDEntry = tab.add("Pickup Extend D", 1).getEntry();
  private GenericEntry extendIZEntry = tab.add("Pickup Extend IZ", 0).getEntry();
  private GenericEntry extendFFEntry = tab.add("Pickup Extend FF", 0).getEntry();
  private GenericEntry extendMinEntry = tab.add("Pickup Extend Min", -.6).getEntry();
  private GenericEntry extendMaxEntry = tab.add("Pickup Extend Max", .6).getEntry();
  private GenericEntry extendSetPointEntry = tab.add("Pickup Extend Set Point", 0.0).getEntry();

  private GenericEntry elbowPEntry = tab.add("Pickup Elbow P", 1).getEntry();
  private GenericEntry elbowIEntry = tab.add("Pickup Elbow I", 1e-4).getEntry();
  private GenericEntry elbowDEntry = tab.add("Pickup Elbow D", 1).getEntry();
  private GenericEntry elbowIZEntry = tab.add("Pickup Elbow IZ", 0).getEntry();
  private GenericEntry elbowFFEntry = tab.add("Pickup Elbow FF", 0).getEntry();
  private GenericEntry elbowMinEntry = tab.add("Pickup Elbow Min", -.5).getEntry();
  private GenericEntry elbowMaxEntry = tab.add("Pickup Elbow Max", .5).getEntry();
  private GenericEntry elbowSetPointEntry = tab.add("Pickup Elbow Set Point", 0.0).getEntry();

  public enum ElbowDirection { STOP, RAISE, LOWER };
  public enum StingerDirection { STOP, EXTEND, RETRACT };
  public enum ShoulderState { RAISED, LOWERED };
  public enum GrabberState { PINCH, DROP };

  /** Creates a new Stinger. */
  public Stinger() {
    m_stingerSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kStingerSolenoid);
    m_grabberSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kGrabbySolenoidIndex);

    m_shoulderInPlace = new DigitalInput(0);

    m_extendSpark = new CANSparkMax(Constants.kExtendStingerSpark, MotorType.kBrushless);
    m_elbowSpark = new CANSparkMax(Constants.kRaiseStingerSpark, MotorType.kBrushless);

    m_extendSpark.setIdleMode(IdleMode.kBrake);
    m_elbowSpark.setIdleMode(IdleMode.kBrake);

    m_extendController = m_extendSpark.getPIDController();
    m_extendEncoder = m_extendSpark.getEncoder();
    m_extendController.setP(kExtendP);
    m_extendController.setI(kExtendI);
    m_extendController.setD(kExtendD);
    m_extendController.setIZone(kExtendIZ);
    m_extendController.setFF(kExtendFF);
    m_extendController.setOutputRange(kExtendMin, kExtendMax);

    m_elbowController = m_elbowSpark.getPIDController();
    m_elbowEncoder = m_elbowSpark.getEncoder();
    m_elbowController.setP(kElbowP);
    m_elbowController.setI(kElbowI);
    m_elbowController.setD(kElbowD);
    m_elbowController.setIZone(kElbowIZ);
    m_elbowController.setFF(kElbowFF);
    m_elbowController.setOutputRange(kElbowMin, kElbowMax);

    m_shoulderState = m_stingerSolenoid.get() ? ShoulderState.RAISED : ShoulderState.LOWERED;
    m_grabberState = m_grabberSolenoid.get() ? GrabberState.PINCH : GrabberState.DROP;

    SmartDashboard.putNumber("Elbow Speed", 0.25);
    SmartDashboard.putNumber("Extend Speed", 0.6);
    SmartDashboard.putData("Zero Encoders", new InstantCommand(() -> {
      stopElbow();
      stopExtend();
      m_elbowSpark.getEncoder().setPosition(0);
      setElbowSetPoint(0);
      m_extendSpark.getEncoder().setPosition(0);
      setExtendSetPoint(0);
    }));
    m_elbowEncoder.setPosition(3.92);
    // tab.add("Pickup Elbow position", 16);
  }

  public double elbowSpeed = 0.0;
  public double extendSpeed = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowSpeed = SmartDashboard.getNumber("Elbow Speed", 0.25);
    extendSpeed = SmartDashboard.getNumber("Extend Speed", 0.6);

    // double p = SmartDashboard.getNumber("Pickup elbow p", 0);
    // double i = SmartDashboard.getNumber("Pickup elbow i", 0);
    // double d = SmartDashboard.getNumber("Pickup elbow d", 0);
    // double iz = SmartDashboard.getNumber("Pickup elbow iz", 0);
    // double ff = SmartDashboard.getNumber("Pickup elbow ff", 0);
    double p = extendPEntry.getDouble(kExtendP);
    double i = extendIEntry.getDouble(kExtendI);
    double d = extendDEntry.getDouble(kExtendD);
    double iz = extendIZEntry.getDouble(kExtendIZ);
    double ff = extendFFEntry.getDouble(kExtendFF);
    double min = extendMinEntry.getDouble(-.25);
    double max = extendMaxEntry.getDouble(.25);
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
    if (min != kExtendMin) {
      kExtendMin = min;
      m_extendController.setOutputRange(kExtendMin, kExtendMax);
    }
    if (max != kExtendMax) {
      kExtendMax = max;
      m_extendController.setOutputRange(kExtendMin, kExtendMax);
    }

    p = elbowPEntry.getDouble(kElbowP);
    i = elbowIEntry.getDouble(kElbowI);
    d = elbowDEntry.getDouble(kElbowD);
    iz = elbowIZEntry.getDouble(kElbowIZ);
    ff = elbowFFEntry.getDouble(kElbowFF);
    min = elbowMinEntry.getDouble(-.25);
    max = elbowMaxEntry.getDouble(.25);
    if (p != kElbowP) {
      m_elbowController.setP(p);
      kElbowP = p;
    }
    if (i != kElbowI) {
      m_elbowController.setI(i);
      kElbowD = i;
    }
    if (d != kElbowD) {
      m_elbowController.setD(d);
      kElbowD = d;
    }
    if (iz != kElbowIZ) {
      m_elbowController.setI(iz);
      kElbowIZ = iz;
    }
    if (ff != kElbowFF) {
      m_elbowController.setI(ff);
      kElbowFF = ff;
    }
    if (min != kElbowMin) {
      kElbowMin = min;
      m_elbowController.setOutputRange(kElbowMin, kElbowMax);
    }
    if (max != kElbowMax) {
      kElbowMax = max;
      m_elbowController.setOutputRange(kElbowMin, kElbowMax);
    }
    
    if(!isShoulderDown() && m_elbowEncoder.getPosition() > 60 && elbowDirection == ElbowDirection.RAISE) {
      setElbow(ElbowDirection.STOP);
    }

    if(m_extendEncoder.getPosition() > 700 && stingerDirection == StingerDirection.EXTEND) {
      setExtend(StingerDirection.STOP);
    }

    

    // kExtendSetpoint = extendSetPointEntry.getDouble(0);
    // kElbowSetpoint = elbowSetPointEntry.getDouble(0);

    // if(isExtendFinished()) {
    //   stopExtend();
    // } else {
    //   enableExtendClosedLoop();
    // }

    // if(isElbowFinished()) {
    //   stopElbow();
    // } else {
    //   enableElbowClosedLoop();
    // }

    SmartDashboard.putNumber("Elbow Position", m_elbowSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Extend Position", m_extendSpark.getEncoder().getPosition());
  }

  public void enableExtendClosedLoop() {
    // kExtendSetpoint = extendSetPointEntry.getDouble(0);
    m_extendController.setReference(kExtendSetpoint, CANSparkMax.ControlType.kPosition);
  }

  public void setExtendSetPoint(double setPoint) {
    kExtendSetpoint = setPoint;
    extendSetPointEntry.setDouble(setPoint);
  }

  public boolean isExtendFinished() {
    return Math.abs(m_extendSpark.getEncoder().getPosition() - kExtendSetpoint) <= kExtendDeadband;
  }

  public void enableElbowClosedLoop() {
    // kElbowSetpoint = elbowSetPointEntry.getDouble(0);
    m_elbowController.setReference(kElbowSetpoint, CANSparkMax.ControlType.kPosition);
  }

  public void setElbowSetPoint(double setPoint) {
    kElbowSetpoint = setPoint;
    elbowSetPointEntry.setDouble(kElbowSetpoint);
  }

  public boolean isElbowFinished() {
    return Math.abs(m_elbowSpark.getEncoder().getPosition() - kElbowSetpoint) <= kElbowDeadband;
  }
  
  public void setGrabber(GrabberState state) {
    m_grabberState = state;
    m_grabberSolenoid.set(state == GrabberState.PINCH);
  }

  public void toggleGrabber() {
    setGrabber(m_grabberState == GrabberState.DROP ? GrabberState.PINCH : GrabberState.DROP);
  }

  public void setElbow(ElbowDirection direction) {
    elbowDirection = direction;
    m_elbowSpark.set(direction == ElbowDirection.STOP ? 0
      : direction == ElbowDirection.LOWER ? -elbowSpeed : elbowSpeed);
  }

  public void stopElbow() {
    m_elbowSpark.set(0);
  }

  public void setExtend(StingerDirection direction) {
    stingerDirection = direction;

    double moveSpeed = extendSpeed;
    if(direction == StingerDirection.RETRACT) {
      moveSpeed *= -1;
      if(m_extendEncoder.getPosition() < 150) moveSpeed *= 0.5;
    }
    if(direction == StingerDirection.STOP) moveSpeed = 0;

    m_extendSpark.set(moveSpeed);
  }

  public void stopExtend() {
    m_extendSpark.set(0);
  }

  public boolean isShoulderDown() {
    return !m_shoulderInPlace.get();
  }

  public void setShoulder(ShoulderState state) {
    m_shoulderState = state;
    m_stingerSolenoid.set(state == ShoulderState.RAISED);
  }

  public void toggleShoulder() {
    setShoulder(m_shoulderState == ShoulderState.RAISED ? ShoulderState.LOWERED : ShoulderState.RAISED);
  }
}
