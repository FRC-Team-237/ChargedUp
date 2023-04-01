// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimeLightInterface {
    String m_limelightAddress; 
    
    public LimeLightInterface(String address){
        m_limelightAddress = address; 
    }
    public double getTHor() {
        return NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("thor").getDouble(0);
    }
    public double getTX(){
        return NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("tx").getDouble(0);
    }
    public double getTY(){
        return NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("ty").getDouble(0);
    }
    public double getTV(){
        return NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("tv").getDouble(0);
    }
    public boolean hasTarget() {
        return (getTV() >= 1.0); 
    }
    public double getPipline(){
        return NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("getpipe").getDouble(0);
    }
    public void setPipeline(int pipeIndex){
        NetworkTableInstance.getDefault().getTable(m_limelightAddress).getEntry("pipeline").setNumber(pipeIndex);
    }   

}
