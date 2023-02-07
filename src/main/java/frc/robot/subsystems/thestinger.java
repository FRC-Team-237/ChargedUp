// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSWDLJNI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class thestinger extends SubsystemBase {
  /** Creates a new thestinger. */
  private CANSparkMax m_stingerMotor1;
private CANSparkMax m_stingerMotor2;
  public thestinger() {
    m_stingerMotor1 = new CANSparkMax(0, MotorType.kBrushless);
    m_stingerMotor2 = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

