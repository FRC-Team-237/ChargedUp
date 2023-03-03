package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ConversionHelper;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Pincher.PinchState;
import frc.robot.subsystems.Stinger.GrabberState;
import frc.robot.subsystems.Stinger.ShoulderState;

public class PickupPosition extends CommandBase {

    Stinger m_stinger;
    Pincher m_pincher;

    public PickupPosition(Stinger stinger, Pincher pincher) {
        m_stinger = stinger;
        m_pincher = pincher;
        addRequirements(stinger, pincher);
    }

    @Override
    public void initialize() {
      m_stinger.setShoulder(ShoulderState.LOWERED);
      m_stinger.setGrabber(GrabberState.DROP);

      m_stinger.setElbowSetPoint(18);
      m_stinger.setExtendSetPoint(175);
      m_stinger.enableElbowClosedLoop();
      m_stinger.enableExtendClosedLoop();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
      return (
        ConversionHelper.inPosition(m_stinger.m_elbowEncoder.getPosition(), m_stinger.kElbowSetpoint, 0.5) &&
        ConversionHelper.inPosition(m_stinger.m_extendEncoder.getPosition(), m_stinger.kExtendSetpoint, 5) &&
        m_stinger.isShoulderDown()
      );
        // return m_stinger.isElbowFinished() && m_stinger.isExtendFinished() && m_stinger.isShoulderDown();
    }
}
