// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

  public enum ElbowDirection { RAISE, LOWER, STOP };
  public enum StingerDirection { EXTEND, RETRACT, STOP };
  public enum ShoulderState { RAISED, LOWERED };
  public enum GrabberState { PINCH, DROP };
 
  /** Creates a new Stinger. */
  public Stinger() {
    m_stingerSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kStingerSolenoid);
    m_grabberSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kGrabbySolenoidIndex);

    m_extendSpark = new CANSparkMax(Constants.kExtendStingerSpark,MotorType.kBrushless);
    m_raiseSpark = new CANSparkMax(Constants.kRaiseStingerSpark,MotorType.kBrushless);

    m_extendSpark.setIdleMode(IdleMode.kBrake);
    m_raiseSpark.setIdleMode(IdleMode.kBrake);

    m_shoulderState = m_stingerSolenoid.get() ? ShoulderState.RAISED : ShoulderState.LOWERED;

    m_grabberState = m_grabberSolenoid.get() ? GrabberState.PINCH : GrabberState.DROP;

    SmartDashboard.putNumber("Elbow Speed", 0.25);
    SmartDashboard.putNumber("Extend Speed", 0.25);
  }

  private double elbowSpeed = 0.0;
  private double extendSpeed = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowSpeed = SmartDashboard.getNumber("Elbow Speed", 0.25);
    extendSpeed = SmartDashboard.getNumber("Extend Speed", 0.25);
    SmartDashboard.putNumber("Elbow Position", m_raiseSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Extend Position", m_extendSpark.getEncoder().getPosition());
  }
  
  public void setGrabber(GrabberState state) {
    m_grabberState = state;
    m_grabberSolenoid.set(state == GrabberState.PINCH);
  }

  public void setElbow(ElbowDirection direction) {
    m_raiseSpark.set(direction == ElbowDirection.STOP ? 0
      : direction == ElbowDirection.LOWER ? elbowSpeed : -elbowSpeed);
  }

  public void setStinger(StingerDirection direction) {
    m_extendSpark.set(direction == StingerDirection.STOP ? 0
      : direction == StingerDirection.EXTEND ? extendSpeed : -extendSpeed);
  }

  public void setShoulder(ShoulderState state) {
    m_shoulderState = state;
    m_stingerSolenoid.set(state == ShoulderState.RAISED);
  }

  public void toggleShoulder() {
    setShoulder(m_shoulderState == ShoulderState.RAISED ? ShoulderState.LOWERED : ShoulderState.RAISED);
  }
}
