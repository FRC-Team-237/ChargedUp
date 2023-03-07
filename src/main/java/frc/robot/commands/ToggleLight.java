package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleLight extends InstantCommand {

  private NetworkTable limelightTable;

  private boolean toggleOn;

  public ToggleLight(boolean toggleOn) {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    this.toggleOn = toggleOn;
  }

  @Override
  public void initialize() {
    limelightTable.getEntry("ledMode").setNumber(toggleOn ? 3 : 1);
  }
}
