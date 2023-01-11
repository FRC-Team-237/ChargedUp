// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;


/** Add your docs here. */
public class TalonRamseteControllerAbstraction extends RamseteController {
    private Pose2d m_poseError; 

    private final double m_b; 
    private final double m_zeta; 

    public TalonRamseteControllerAbstraction(double b, double zeta){
        super(b, zeta);
        m_b = b; 
        m_zeta = zeta; 

    }
    public TalonRamseteControllerAbstraction() {
        super(2.0,0.7);
        this.m_b = 2.0;
        this.m_zeta = 0.7; 
    }
    private static double sinc(double x){
        if (Math.abs(x) < 1e-9){
            return 1.0 - 1.0 /6.0 * x * x; 
        }
        else {
            return Math.sin(x)/x; 
        }
    }
    @Override 
    public ChassisSpeeds calculate(Pose2d currentPose,
    Pose2d poseRef,
    double linearVelocityRefMeters,
    double angularVelocityRefRadiansPerSecond
    ){
        this.m_poseError = poseRef.relativeTo(currentPose);
        // equations for clarity 
        final double eX = m_poseError.getTranslation().getX();  
        final double eY = m_poseError.getTranslation().getY();
        final double eTheta = m_poseError.getRotation().getRadians(); 
        final double vRef = linearVelocityRefMeters; 
        final double omegaRef = angularVelocityRefRadiansPerSecond; 

        double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2)); 
        return new ChassisSpeeds( vRef * m_poseError.getRotation().getCos() + k * eX,
        0.0,
        omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY); 
    }
    public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState){
        return calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter); 
    }
}
