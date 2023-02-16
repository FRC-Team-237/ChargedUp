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
  private boolean m_lowered;
  private CANSparkMax m_extendSpark;
  private CANSparkMax m_raiseSpark;
 
  /** Creates a new Stinger. */
  public Stinger() {
    m_stingerSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kStingerSolenoid);
    m_lowered = false;
    m_extendSpark=new CANSparkMax(Constants.kExtendStingerSpark,MotorType.kBrushless);
    m_extendSpark.setIdleMode(IdleMode.kBrake);
    m_raiseSpark=new CANSparkMax(Constants.kRaiseStingerSpark,MotorType.kBrushless);
    m_raiseSpark.setIdleMode(IdleMode.kBrake);

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
  public void raiseElbow(){
    m_raiseSpark.set(-elbowSpeed);
  }
  public void lowerElbow(){
    m_raiseSpark.set(elbowSpeed);
  }
  public void stopElbow(){
    m_raiseSpark.set(0);
  }
  public void extendStinger(){
    m_extendSpark.set(extendSpeed);
  }
  public void retractStinger(){
    m_extendSpark.set(-extendSpeed);
  }
  public void stopStinger(){
    m_extendSpark.set(0);
  }
  public void lowerStinger() {
    m_stingerSolenoid.set(false);
    m_lowered = true;
  }

  public void raiseStinger() {
    m_stingerSolenoid.set(true);
    m_lowered = false;
  }

  public void toggleStinger() {
    m_lowered = !m_lowered;
    m_stingerSolenoid.set(m_lowered);
  }
}
