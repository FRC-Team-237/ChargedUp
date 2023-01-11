// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/** Add your docs here. */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

private CANSparkMax shooterMotor1; 
private CANSparkMax shooterMotor2;
private SparkMaxPIDController m_PidController1;
private SparkMaxPIDController m_PidController2;
public double kP,kI,kD,kIZ, kFF,kMaxOutput, kMinOutput, kMaxRPM;
private Solenoid m_shootySolenoid;
private DigitalInput m_shooterSensor;
private DigitalInput m_totSwitch;
private boolean m_isRunning;
private Debouncer m_totDebouncer; 
private double m_setPoint1;

public Shooter(){

  shooterMotor1 = new CANSparkMax(Constants.kShooterSpark1, MotorType.kBrushless);
  shooterMotor1.setInverted(false);
  shooterMotor1.setClosedLoopRampRate(0.25);
  shooterMotor2 = new CANSparkMax(Constants.kShooterSpark2, MotorType.kBrushless);
  shooterMotor2.setInverted(false);
  shooterMotor2.setClosedLoopRampRate(0.25);
  m_PidController1 = shooterMotor1.getPIDController();
  m_PidController2 = shooterMotor2.getPIDController();
  //kP = 5e-5;
  // kP = 0.01;
  // kI = 0.0;
  // kD = 1.0;
  kP = 0.0;
  kI = 0.0;
  kD = 0.0;
  //kI = 3e-7;
  //kD = 0.008;
  kIZ = 0;
  // kFF = 0.01;
  kFF = 0.0002;
  kMaxOutput = 1;
  kMinOutput = -1;
  kMaxRPM = 5700;

  m_setPoint1 = Constants.kShooter1SetPoint;

  m_PidController1.setP(kP);
  m_PidController1.setI(kI);
  m_PidController1.setD(kD);
  m_PidController1.setIZone(kIZ);
  m_PidController1.setFF(kFF);
  m_PidController1.setOutputRange(kMinOutput, kMaxOutput);

  m_PidController2.setP(kP);
  m_PidController2.setI(kI);
  m_PidController2.setD(kD);
  m_PidController2.setIZone(kIZ);
  m_PidController2.setFF(kFF);
  m_PidController2.setOutputRange(kMinOutput, kMaxOutput);

  m_shootySolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kShootySolenoidIndex);
  m_shooterSensor = new DigitalInput(Constants.kSenseyShooty);
  m_totSwitch = new DigitalInput(Constants.kTOTSwitch);
  m_totDebouncer = new Debouncer(0.5,DebounceType.kBoth);

  SmartDashboard.putNumber("Shooter P", kP);
  SmartDashboard.putNumber("Shooter I", kI);
  SmartDashboard.putNumber("Shooter D", kD);
  SmartDashboard.putNumber("ShooterFF", kFF);
  SmartDashboard.putNumber("Big Wheel Set Point", m_setPoint1);
}
  @Override 
  public void periodic(){
    SmartDashboard.putNumber("Big Wheel Speed", shooterMotor1.getEncoder().getVelocity()); 
    SmartDashboard.putNumber("Big Wheel Motor", shooterMotor1.get());
    SmartDashboard.putNumber("Small Wheel Speed", shooterMotor2.getEncoder().getVelocity()); 
    SmartDashboard.putNumber("Small Wheel Motor", shooterMotor2.get());
    SmartDashboard.putBoolean("TOT Raised", isTOTRaised());

    kP = SmartDashboard.getNumber("Shooter P", 0.0);
    kI = SmartDashboard.getNumber("Shooter I", 0.0);
    kD = SmartDashboard.getNumber("Shooter D", 0.0);
    kFF = SmartDashboard.getNumber("ShooterFF", 0.0002);
    m_setPoint1 = SmartDashboard.getNumber("Big Wheel Set Point", Constants.kShooter1SetPoint);

    m_PidController1.setP(kP);
    m_PidController1.setI(kI);
    m_PidController1.setD(kD);
    m_PidController1.setFF(kFF);

    m_PidController2.setP(kP);
    m_PidController2.setI(kI);
    m_PidController2.setD(kD);
    m_PidController2.setFF(kFF);
    //idleSpin();
  }
  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
  }
  public void idleSpin(){
    if (!m_isRunning) {
      if(isBallReady()) {
         shooterMotor1.set(Constants.kIdleSpeed);
        //m_pPidController.setReference(Constants.kIdleSpeed, ControlType.kVelocity);
      } else {
        offWheel();
      }
    }
  }

  public void onWheel(){
    m_PidController1.setReference(m_setPoint1, ControlType.kVelocity);
    shooterMotor2.set(1.0);
    m_isRunning = true;
    //shooterMotor.set(0.1); 
  }

  public void offWheel(){
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
    m_isRunning = false;
  }

  public double getWheelSpeed1(){
    return shooterMotor1.getEncoder().getVelocity();
  }

  public double getWheelSpeed2(){
    return shooterMotor2.getEncoder().getVelocity();
  }

  public void raiseTOT(boolean raise){
    m_shootySolenoid.set(raise);
  }

  public boolean isBallReady() {
    return m_totDebouncer.calculate(m_shooterSensor.get());
  }
  public void reverse(){
    m_PidController1.setReference(Constants.kreverseSetPoint, ControlType.kVelocity);
    m_isRunning = true;
    //shooterMotor.set(-0.1);
  }

  public boolean isTOTRaised() {
    return !m_totSwitch.get();
  }
}
