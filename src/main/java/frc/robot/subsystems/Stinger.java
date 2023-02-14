// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
    m_raiseSpark=new CANSparkMax(Constants.kRaiseStingerSpark,MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void raiseElbow(){
  m_raiseSpark.set(.25);
}
public void lowerElbow(){
  m_raiseSpark.set(-.25);
}
public void stopElbow(){
  m_raiseSpark.set(0);
}
public void extendStinger(){
  m_extendSpark.set(.25);
}
public void retractStinger(){
  m_extendSpark.set(-.25);
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
