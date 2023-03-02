package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Pincher.DropState;
import frc.robot.subsystems.Stinger.ShoulderState;

public class DrivePosition extends CommandBase {

    Stinger m_stinger;
    Pincher m_pincher;

    public DrivePosition(Stinger stinger, Pincher pincher) {
        m_stinger = stinger;
        m_pincher = pincher;
        addRequirements(stinger, pincher);
    }

    @Override
    public void initialize() {
      m_stinger.setShoulder(ShoulderState.RAISED);
      m_pincher.setDropper(DropState.RAISED);
      m_stinger.setExtendSetPoint(25);
      m_stinger.setElbowSetPoint(16.85);
      m_stinger.enableElbowClosedLoop();
      m_stinger.enableExtendClosedLoop();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return true;
    }
}
