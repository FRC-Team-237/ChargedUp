// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // private CANSparkMax m_intakeMotor1;
  // private CANSparkMax m_intakeMotor2;
  private Solenoid m_intakeSolenoid;
  private Solenoid m_droppySolenoid;
  private boolean m_islowered;



  /** Creates a new Intake. */
  public Intake() {
    // m_intakeMotor1 = new CANSparkMax(motorCANId1, MotorType.kBrushless);
    // m_intakeMotor1.setIdleMode(IdleMode.kCoast);
    // m_intakeMotor2 = new CANSparkMax(motorCANId2, MotorType.kBrushless);
    // m_intakeMotor2.setIdleMode(IdleMode.kCoast);
    m_islowered=false;
    m_intakeSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kGrabbySolenoidIndex);
    m_droppySolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kdroppySolenoidIndex);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void closeIntake() {
    m_intakeSolenoid.set(true);
  }

  public void openIntake() {
    m_intakeSolenoid.set(false);
  }

  public void lowerIntake() {
    m_droppySolenoid.set(true);
    m_islowered=true;
  }

  public void raiseIntake() {
    m_droppySolenoid.set(false);
    m_islowered=false;
  }
  public void raisevlow(){
    m_islowered=!m_islowered;
    m_droppySolenoid.set(m_islowered);
  
  }

  public void stopIntake() {
    // m_intakeMotor1.set(0.0);
    // m_intakeMotor2.set(0.0);
  }
}
