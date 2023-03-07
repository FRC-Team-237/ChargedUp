package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ConversionHelper;
import frc.robot.subsystems.DriveTrain;

public class TargetPegPID extends CommandBase {

  private DriveTrain driveTrain;
  private boolean isMid;
  private boolean alsoDrive;

  PIDController drivePID;
  PIDController rotationPID;

  private NetworkTable limelightTable;

  private Debouncer rotateDebouncer;

  private double midTargetArea = 0.22;
  private double midAreaTolerance = 0.05;
  private double topTargetArea = 0.08;
  private double topAreaTolerance = 0.05;

  private double tx;
  private double ta;
  private double tv;

  private double targetP;
  private double targetI;
  private double targetD;
  private double driveP;
  private double driveI;
  private double driveD;

  public TargetPegPID(DriveTrain driveTrain, boolean isMid, boolean alsoDrive) {

    rotateDebouncer = new Debouncer(0.5, DebounceType.kBoth);

    addRequirements(driveTrain);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    this.isMid = isMid;
    this.drivePID = new PIDController(0, 0, 0);
    this.rotationPID = new PIDController(0, 0, 0);
    
    this.driveTrain = driveTrain;
    rotationPID.setTolerance(2);
    
    drivePID.setTolerance(isMid ? midAreaTolerance : topAreaTolerance);
    drivePID.setSetpoint(isMid ? midTargetArea : topTargetArea);

    double[] cropValues = new double[4];
    if(isMid) {
      // Crop mid below 0.57
      cropValues[0] = -1.0;
      cropValues[1] = 1.0;
      cropValues[2] = -1.0;
      cropValues[3] = 0.57;
    } else {
      // Crop top above 0.57
      cropValues[0] = -1.0;
      cropValues[1] = 1.0;
      cropValues[2] = 0.57;
      cropValues[3] = 1.0;
    }
    limelightTable.getEntry("crop").setDoubleArray(cropValues);
  }

  @Override
  public void initialize() {
    targetP = SmartDashboard.getNumber("Target P", 0.05);
    targetI = SmartDashboard.getNumber("Target I", 0);
    targetD = SmartDashboard.getNumber("Target D", 0.0025);
    rotationPID.setPID(targetP, targetI, targetD);

    driveP = SmartDashboard.getNumber("Drive P", 0);
    driveP = SmartDashboard.getNumber("Drive I", 0);
    driveP = SmartDashboard.getNumber("Drive D", 0);
    drivePID.setPID(driveP, driveI, driveD);
  }

  @Override
  public void execute() {
    tx = limelightTable.getEntry("tx").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);

    double rotation = -rotationPID.calculate(tx);
    rotation = Math.min(Math.max(rotation, -0.25), 0.25);
    double drive = drivePID.calculate(ta);
    drive = Math.min(Math.max(drive, -0.25), 0.25);
    if(alsoDrive) {
      drive = drivePID.calculate(ta);
    }

    driveTrain.driveRaw(rotation, drive);
  }

  @Override
  public boolean isFinished() {
      return (rotateDebouncer.calculate(rotationPID.atSetpoint()) && drivePID.atSetpoint())
      || tv < 1;
  }
}