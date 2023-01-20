// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intakeMotor1;
  private CANSparkMax m_intakeMotor2;

  /** Creates a new Intake. */
  public Intake(int motorCANId1, int motorCANId2) {
    m_intakeMotor1 = new CANSparkMax(motorCANId1, MotorType.kBrushed);
    m_intakeMotor1.setIdleMode(IdleMode.kBrake);
    m_intakeMotor2 = new CANSparkMax(motorCANId2, MotorType.kBrushed);
    m_intakeMotor2.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIntake() {
    m_intakeMotor1.set(1.0);
    m_intakeMotor2.set(1.0);
  }

  public void stopIntake() {
    m_intakeMotor1.set(0.0);
    m_intakeMotor2.set(0.0);
  }
}
