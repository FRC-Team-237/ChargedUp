package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ConversionHelper;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Stinger.GrabberState;
import frc.robot.subsystems.Stinger.ShoulderState;

public class PickupPosition extends CommandBase {

    public static boolean isRunning = false;
    Stinger m_stinger;
    Pincher m_pincher;
    private Debouncer shoulderDebouncer;

    public PickupPosition(Stinger stinger) {
        m_stinger = stinger;
        addRequirements(stinger);

        this.shoulderDebouncer = new Debouncer(0.25, DebounceType.kBoth);
    }

    @Override
    public void initialize() {
      PickupPosition.isRunning = true;
      m_stinger.setShoulder(ShoulderState.LOWERED);
      m_stinger.setGrabber(GrabberState.DROP);

      m_stinger.setElbowSetPoint(18);
      m_stinger.setExtendSetPoint(165);
      m_stinger.enableElbowClosedLoop();
      m_stinger.enableExtendClosedLoop();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
      PickupPosition.isRunning = false;
    }

    @Override
    public boolean isFinished() {
      return (
        ConversionHelper.inPosition(m_stinger.m_elbowEncoder.getPosition(), m_stinger.kElbowSetpoint, 0.5) &&
        ConversionHelper.inPosition(m_stinger.m_extendEncoder.getPosition(), m_stinger.kExtendSetpoint, 5) &&
        shoulderDebouncer.calculate(m_stinger.isShoulderDown())
      );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelIncoming;
    }
}
