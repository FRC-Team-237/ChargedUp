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

  private CANSparkMax m_intakeMotor1;
  private CANSparkMax m_intakeMotor2;
  private Solenoid m_intakeSolenoid;

  /** Creates a new Intake. */
  public Intake(int motorCANId1, int motorCANId2) {
    m_intakeMotor1 = new CANSparkMax(motorCANId1, MotorType.kBrushless);
    m_intakeMotor1.setIdleMode(IdleMode.kCoast);
    m_intakeMotor2 = new CANSparkMax(motorCANId2, MotorType.kBrushless);
    m_intakeMotor2.setIdleMode(IdleMode.kCoast);
    m_intakeSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kGrabbySolenoidIndex);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIntake() {
    m_intakeMotor1.set(0.5);
    m_intakeMotor2.set(-0.5);
  }

  public void startIntakeReverse() {
    m_intakeMotor1.set(-0.5);
    m_intakeMotor2.set(0.5);
  }

  public void closeIntake() {
    m_intakeSolenoid.set(true);
  }

  public void openIntake() {
    m_intakeSolenoid.set(false);
  }

  public void stopIntake() {
    m_intakeMotor1.set(0.0);
    m_intakeMotor2.set(0.0);
  }
}
