// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kMaxElbowPosition = 64;
    public static final double kMaxExtendPosition = 312;
    // CAN Addresses
    public static final int kMotorFL = 1;
    public static final int kMotorBL = 2;
    public static final int kMotorFR = 3;
    public static final int kMotorBR = 4;
    public static final int kPCM = 52;
    public static final int kShooterSpark1 = 7;
    // public static final int kShooterSpark2 = 10;
    public static final int kExtendStingerSpark = 8;
    public static final int kRaiseStingerSpark = 9;
    public static final int kPincherSparkLeft = 10;
    public static final int kPincherSparkRight = 11;

    /** The upper limit at which the drive will stop using kDriveReduction */
    public static final double kDriveThreshold = 0.9;
    /** Drive motor speed is multiplied by this value */                                               
    public static final double kDriveReduction = 0.75;
    
    public static final int kGrabbySolenoidIndex = 4;
    public static final int kPincherSolenoidIndex = 7;
    public static final int kStingerSolenoid = 6;
    public static final int kBrakeSolenoid = 3;
    public static final int kdroppySolenoidIndex = 5;
    public static final int kSenseyGrabby = 0;
    public static final int kSenseyShooty = 1;
    public static final int kPincherRaised = 5;
    public static final int kTOTSwitch = 6;
    public static final double kShooter1SetPoint = 3300;
    public static final int kShooter2SetPoint = 5000;
    public static final int kreverseSetPoint = -1000;
    public static final int kslowSpeed = 1500;
    public static final double kIdleSpeed = 0.1;
    public static final int kBlinkinID = 0;

    // control panel buttons 
    public static final int kAutoSwitchOne = 9;
    public static final int kAutoSwitchTwo = 10;
    public static final int kAutoSwitchThree = 11;
    public static final int kAautoSwitchFour = 12;
    public static final int kShoot = 13;
    public static final int kManualShoot = 14;
    public static final int kKillSwitch = 15;
    public static final int kHangOneUp = 2;
    public static final int kHangOnedown = 16;
    public static final int kHangTwoUp = 6;
    public static final int kHangTwoDown = 7;
    public static final int kAutoPickup = 8;
    public static final int kEject = 1;
    public static final int kHangOverride = 4;
    public static final int kAutoBallSwitch = 5;
    public static final int kBalanceSwitch = -1;
    

     public final class DTConsts {
        public static final double kWheelDiameter = 0.50; // Feet 
        public static final double kTrackWidth = 0.0;
        public static final int kTicksPerRevolution = (int)(4096.0*10.71);
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double MAX_VELOCITY = 13.75;
        public static final double MAX_ACCELERATION = 0;
        public static final double kD = 0.0; 
        public static final double kP = 0.0; 
        public static final double kI = 0.0; 
        public static final double kFF = 0.0; 
        public static final int kTimeOut = 10; 
        public static final int kClosedLoopError = 0; 
        public static final int kStatusFrame = 10; 
        public static final double kRampRate = 0.25; 
    }
    public final class LimeLight {
        public static final double kDriveP = 0.45;
        public static final double kSteerP = 0.03;
        public static final double kGoalSteerP = 0.02;
        public static final double kGoalDriveP = 0.05;
        public static final double kDesiredTarget = 15.0;
        public static final double kMaxDrive = 0.65;
        public static final double kGoalMaxDrive = 0.5;
        public static final double kMinSpeed = 0.05; 
    }
    public final class Colors {
        public static final double kBlue = 0.3;
        public static final double kGreen = 0.85;
        public static final double kRed = 0.61;
        public static final double kOneBall = 0.15;
        public static final double kTwoBalls = 0.35;
        public static final double kShoot = 0.93;
        public static final double kHang = 0.91;
    }
    public final class Pickup {
        public static final double kElbow = 16;
        public static final double kExtend = 181;
    }
    public final class ScoreLow {
        public static final double kElbow = 41.8;
        public static final double kExtend = 320;
    }
    public final class ScoreLowPlace {
        public static final double kElbow = 32;
        public static final double kExtend = 392;
    }
    public final class ScoreHigh {
        public static final double kElbow = 54;
        public static final double kElbowAfterLean = 73;
        public static final double kExtend = 320;
    }
    public final class ScoreHighPlace {
        public static final double kElbow = 66;
        public static final double kExtend = 261;
    }
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DTConsts.kTrackWidth);
    
    
}

