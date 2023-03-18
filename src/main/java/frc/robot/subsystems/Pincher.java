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

public class Pincher extends SubsystemBase {
  private Solenoid m_intakeSolenoid;
  private Solenoid m_droppySolenoid;
  public DropState m_dropState;
  private CANSparkMax m_leftPuller;
  private CANSparkMax m_rightPuller;
  private PullState pullState;

  public enum PinchState { OPEN, CLOSED };
  public enum DropState { RAISED, LOWERED };
  public enum PullState { STOP, PUSH, PULL };

  /** Creates a new Pincher. */
  public Pincher() {
    m_intakeSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kPincherSolenoidIndex);
    m_droppySolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kdroppySolenoidIndex);
    m_dropState = m_droppySolenoid.get() ? DropState.LOWERED : DropState.RAISED;

    // m_leftPuller = new CANSparkMax(10, MotorType.kBrushless);
    // m_rightPuller = new CANSparkMax(11, MotorType.kBrushless);
    pullState = PullState.STOP;
  }

  @Override
  public void periodic() {}

  public void setPull(PullState state) {
    this.pullState = state;
    if(state == PullState.STOP) {
      // m_leftPuller.set(0);
      // m_rightPuller.set(0);
      return;
    }
    // m_leftPuller.set(state == PullState.PULL ? 0.25 : -0.25);
    // m_rightPuller.set(state == PullState.PULL ? -0.25 : 0.25);
  }

  public void setPincher(PinchState state) {
    if (m_dropState == DropState.RAISED) {
      return;
    }
    m_intakeSolenoid.set(state == PinchState.CLOSED);
  }

  public void setDropper(DropState state) {
    m_dropState = state;
    if (m_dropState == DropState.RAISED) {
      setPincher(PinchState.OPEN);
    }
    m_droppySolenoid.set(state == DropState.LOWERED);
  }

  public void toggleDropper() {
    setDropper(m_dropState == DropState.RAISED ? DropState.LOWERED : DropState.RAISED);
  }
}