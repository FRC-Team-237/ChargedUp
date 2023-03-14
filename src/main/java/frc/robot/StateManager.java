// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Singleton Class to help manage and post states of the robot to the smart dashboard */
public class StateManager extends SubsystemBase{
    private static StateManager m_stateManager = null; 
    private HashMap<String,String> m_stateMap; 
    private StateManager() {
        m_stateMap = new HashMap<String,String>();
    }
    public StateManager getInstance() {
        if (m_stateManager == null){
            m_stateManager = new StateManager(); 
        }
        return m_stateManager; 
    }
    public void updateState(String stateName, String state) {
        m_stateMap.put(stateName, state); 
    }

    @Override
    public void periodic(){
        for (HashMap.Entry<String,String> itr : m_stateMap.entrySet()) {
            SmartDashboard.putString(itr.getKey(), itr.getValue()) ;   
        }
    }
}


