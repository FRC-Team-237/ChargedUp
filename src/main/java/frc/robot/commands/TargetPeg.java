package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TargetPeg extends CommandBase {

    public TargetPeg() {

    }

    @Override
    public void initialize() {
        System.out.println("Target X: " + NetworkTableInstance.getDefault().getTable("limelight-goal").getEntry("tx").getDouble(0));
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
