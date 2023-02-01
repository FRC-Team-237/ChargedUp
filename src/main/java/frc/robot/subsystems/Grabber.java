// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {

  private ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher = new ColorMatch();
  private Solenoid m_grabbySolenoid;
  private Solenoid m_pickupSolenoid;
  private DigitalInput m_grabberSensor;
  private DigitalInput m_grabberRaisedSensor;
  private boolean m_isRedAlliance;
  private Color m_red;
  private Color m_blue;
  private Color m_floor;

  /** Creates a new Grabber. */
  public Grabber() {
    m_grabbySolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, 0);
    m_pickupSolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kPickupSolenoidIndex);
    m_colorSensor = new ColorSensorV3(Port.kOnboard);
    // m_red = new Color(0.475, 0.388, 0.137);
    m_red = new Color(0.334, 0.477, 0.189);
    // m_blue = new Color(0.19, 0.437, 0.372);
    m_blue = new Color(0.261, 0.494, 0.245);
    m_floor = new Color(0.295, 0.5, 0.2);
    // m_colorMatcher.addColorMatch(Color.kBlue);
    // m_colorMatcher.addColorMatch(Color.kGreen);
    // m_colorMatcher.addColorMatch(Color.kRed);
    m_colorMatcher.addColorMatch(m_red);
    m_colorMatcher.addColorMatch(m_blue);
    m_colorMatcher.addColorMatch(m_floor);
    m_grabberSensor = new DigitalInput(Constants.kSenseyGrabby);
    m_grabberRaisedSensor = new DigitalInput(Constants.kGrabberRaised);
    m_isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Color", stringColor());
    SmartDashboard.putBoolean("Ball Present", m_grabberSensor.get());
    SmartDashboard.putBoolean("Grabber Open", !m_grabbySolenoid.get());
  }

  public void setIsRedAlliance(boolean red) {
    m_isRedAlliance = red;
  }

  public String stringColor() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    SmartDashboard.putNumber("Detected Color Red", detectedColor.red);
    SmartDashboard.putNumber("Detected Color Green", detectedColor.green);
    SmartDashboard.putNumber("Detected Color Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    // if (match.color == Color.kRed) {
    //   return "Red";
    // } else if (match.color == Color.kBlue) {
    //   return "Blue";
    // } else {
    //   return "Green";
    // }
    if (match.color == m_red) {
      return "Red";
    } else if (match.color == m_blue) {
      return "Blue";
    } else {
      return "None";
    }
  }

  private boolean isBallRed() {
    return stringColor().equals("Red");
  }

  /** When true, this closes the grabber */
  public void grabbyGrab(boolean grab) {
    m_grabbySolenoid.set(grab);
  }
  
  /** When true, this lowers the grabber */
  public void lowerGrabber(boolean raise){
    m_pickupSolenoid.set(raise);
  }

  public boolean ballAtGrabber(){
    return m_grabberSensor.get();
  }

  public boolean canPickup() {
    if (m_isRedAlliance == isBallRed() && ballAtGrabber()) {
      return true;
    } else {
      return false;
    }
  }
  
  public boolean isClosed() {
    return m_grabbySolenoid.get();
  }

  public boolean grabberRaised() {
    return !m_grabberRaisedSensor.get();
  }
}
