package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ConversionHelper;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TargetPeg extends CommandBase {

    private float xOffset = 0.0f;
    private boolean hasTarget = false;

    private NetworkTable limelightTable;

    private DriveTrain driveTrain;

    private Debouncer steerDebouncer;

    public TargetPeg(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.steerDebouncer = new Debouncer(0.25, DebounceType.kBoth);
    }

    @Override
    public void initialize() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        xOffset = limelightTable.getEntry("tx").getFloat(0);
        hasTarget = limelightTable.getEntry("tv").getFloat(0) >= 1; 
        this.driveTrain.enableMotorBreak();
    }

    @Override
    public void execute() {
        this.xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getFloat(0);
        this.hasTarget = limelightTable.getEntry("tv").getFloat(0) >= 1;

        SmartDashboard.putString("Target Info", "X Offset: " + xOffset + " | " + "Has Target: " + hasTarget);

        double turnValue = xOffset * 0.02;

        driveTrain.driveRaw(turnValue, 0);
    }
    
    @Override
    public void end(boolean interrupted) {
        this.driveTrain.disableMotorBreak();
    }

    @Override
    public boolean isFinished() {
        return !hasTarget
        || steerDebouncer.calculate(ConversionHelper.inPosition(xOffset, 0, 1));
    }
}
