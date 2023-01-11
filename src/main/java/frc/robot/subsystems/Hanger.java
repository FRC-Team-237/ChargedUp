// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Hanger extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_hangerMotor;
  private boolean m_enabled;
  private int m_inverted;
  private int m_hangerNum;
  private double m_forwardsLimit = 0;
  private double m_backwardsLimit = 0;
  private boolean m_limitEnabled = false;

  public Hanger(int canID, boolean inverted) {
    m_hangerMotor = new CANSparkMax(canID, MotorType.kBrushless);
    m_hangerMotor.setIdleMode(IdleMode.kBrake);
    m_enabled = false;
    m_inverted = inverted ? -1 : 1;
    m_hangerNum = canID;
    resetEncoders();
  }
  public void enableHanger() {
    m_enabled = true;
  }
  public void disableHanger() {
    m_enabled = false;
    stopHang();
  }
  public boolean getEnabled() {
    return m_enabled;
  }
  // public void raiseHanger(){
  //   if (!m_enabled) {
  //     return;
  //   }
  //   if ((m_limitEnabled && Math.abs(m_hangerMotor.getEncoder().getPosition()) >= Math.abs(m_forwardsLimit))) {
  //     m_hangerMotor.set(0);
  //     return;
  //   }
  //   m_hangerMotor.set(m_inverted * 0.5);
  // }
  public void raiseHanger(){
    if (!m_enabled) {
      return;
    }
    if (m_limitEnabled) {
      if (
        m_inverted * m_hangerMotor.getEncoder().getPosition() >= m_inverted * m_forwardsLimit
      ) {
        m_hangerMotor.set(0);
        return;
      }
    }
    m_hangerMotor.set(m_inverted * 0.75);
  }
  public void lowerHanger() {
    if (!m_enabled) {
      return;
    }
    if (m_limitEnabled && m_inverted * m_hangerMotor.getEncoder().getPosition() <= m_inverted * m_backwardsLimit) {
      m_hangerMotor.set(0);
      return;
    }
    m_hangerMotor.set(m_inverted * -0.6);
  }
  public void stopHang(){
    m_hangerMotor.set(0.0);
  }
  public double getEncoder() {
    return m_hangerMotor.getEncoder().getPosition();
  }
  public void resetEncoders() {
    m_hangerMotor.getEncoder().setPosition(0);
  }
  public void setForwardsLimit(double limit) {
    m_forwardsLimit = limit;
  }
  public void setBackwardsLimit(double limit) {
    m_backwardsLimit = limit;
  }
  public void enableLimit() {
    m_limitEnabled = true;
  }
  public void disableLimit() {
    m_limitEnabled = false;
  }

  @Override
  public void periodic() {
    if (m_hangerNum == Constants.kHangerOneSpark) {
      SmartDashboard.putNumber("Hanger 1 Encoder", getEncoder());
    } else if (m_hangerNum == Constants.kHangerTwoSpark) {
      SmartDashboard.putNumber("Hanger 2 Encoder", getEncoder());
    }
    if (m_enabled) {
      Leds.getInstance().setColor(Constants.Colors.kHang);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
