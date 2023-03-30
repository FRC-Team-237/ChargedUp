// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class SensorPackage {
    private SensorPackage _instance = null;
    private AnalogInput m_distanceSensor; 
    private LimeLightInterface m_gpLimelight; 
    private LimeLightInterface m_goalLimelight; 
    private SensorPackage(){
        m_distanceSensor = new AnalogInput(0); 
        m_gpLimelight = new LimeLightInterface("limelight-arm");
        m_goalLimelight = new LimeLightInterface("limelight"); 
    }
    public SensorPackage getInstance() {
        if (_instance == null){
            _instance = new SensorPackage(); 
        }
        return _instance; 
    }
    public double getSonarRaw(){
        return m_distanceSensor.getVoltage(); 
    }
    public double getSonarDistance(){
        double sensorValue = m_distanceSensor.getVoltage();
        final double scaleFactor = 1/(5./1024.); //scale converting voltage to distance
        return 5*sensorValue*scaleFactor; //convert the voltage to distance
    }
    public boolean hasGamePieceTarget(){
        return m_gpLimelight.hasTarget(); 
    }
    public boolean hasGoalTarget(){
        return m_goalLimelight.hasTarget(); 
    }

    
}
