// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Leds extends SubsystemBase {
  private static Leds m_leds = null;

  private PWMSparkMax m_blinkin;
  private double m_colorValue;
  private boolean m_isRedAlliance;

  /** Creates a new Leds. */
  private Leds() {
    m_blinkin = new PWMSparkMax(Constants.kBlinkinID);
    m_colorValue = 0.67;
    m_isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
  }

  public static Leds getInstance() {
    if (m_leds == null) {
      m_leds = new Leds();
    }
    return m_leds;
  }

  public void setColor(double colorValue) {
    m_colorValue = colorValue;
  }

  public double getColor() {
    return m_colorValue;
  }

  public void resetColor() {
    Leds.getInstance().setColor(m_isRedAlliance ? Constants.Colors.kRed : Constants.Colors.kBlue);
  }

  public void setIsRedAlliance(boolean isRedAlliance) {
    m_isRedAlliance = isRedAlliance;
  }

  @Override
  public void periodic() {
    m_blinkin.set(m_colorValue);
  }
}
