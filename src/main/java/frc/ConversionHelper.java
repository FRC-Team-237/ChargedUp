// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

/** Add your docs here. */
public class ConversionHelper {
  /**
   * Convert the values from native talon Units (ticks/100ms) to WPI Trajectory Units (m/s)
   * @param talonVelocity
   * @param wheelDiameter in feet
   * @param usingMetric
   * @param ticksPerRev
   * @return WPI units 
   */
  public static final double SRXToTrajectoryUnits(double talonVelocity, double wheelDiameter, boolean usingMetric, int ticksPerRevolution) {
    double result = talonVelocity;
    result = result * 10; //Convert ticks/100ms to ticks/sec
    
    double circumference = 0;

    if (usingMetric) {
      circumference = Math.PI * wheelDiameter; 
    } else {
      double diameterInMeters = toMeters(wheelDiameter); 
      circumference = Math.PI * diameterInMeters; 
    }

    double metersPerTick = circumference / ticksPerRevolution; 
    result = result * metersPerTick; 
    return result;
  }

  /**
   * Convert the values from WPITrajectory Units (m/s) to native Talon units (ticks/100ms)
   * @param metersPerSecond
   * @param wheelDiameter in feet
   * @param givenMetric
   * @param ticksPerRevolution
   * @return Talon Units
   */
  public static final double trajectoryToSRXUnits(double metersPerSecond, double wheelDiameter, boolean givenMetric, int ticksPerRevolution) {
    double result = metersPerSecond;
    double circumference = 0;
    
    if(givenMetric) {
      circumference = Math.PI * wheelDiameter;
    } else {
      double diameterInMeters = toMeters(wheelDiameter);
      circumference = Math.PI * diameterInMeters;
    }

    double ticksPerMeter = ticksPerRevolution/circumference;
    result = result * ticksPerMeter;
    result = result * 0.1;
    
    return result;
  }

  public static double convertTalonEncoderTicksToMeters(int ticks, double wheelDiameter, double ticksPerRevolution, boolean givenMetric) {
    double result = ticks;
    double circumference = 0;

    if(givenMetric) {
      circumference = Math.PI * wheelDiameter;
    } else {
      double diameterInMeters = toMeters(wheelDiameter);
      circumference = Math.PI * diameterInMeters;
    }

    double metersPerTick = circumference / ticksPerRevolution;
    result = result * metersPerTick;
    return result;
  }

  public static double toMeters(double feet) {
    return feet * 0.3048;
  }

  public static double toFeet(double meters) {
    return meters / 0.3048;
  }

  public static double mapRange(double input, double inMin, double inMax, double outMin, double outMax) {
    return (outMax - outMin) * (input - inMin) / (inMax - inMin) + outMin;
  }

  public static boolean inPosition(double value1, double value2, double deadband) {
    return Math.abs(value1 - value2) < deadband;
  }
}