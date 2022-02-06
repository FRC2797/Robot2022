// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static java.lang.Math.tan; 
import static java.lang.Math.toRadians; 

public class Limelight extends SubsystemBase {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-knight");
  private NetworkTableEntry horizontalOffset = table.getEntry("tx");
  private NetworkTableEntry verticalOffset = table.getEntry("ty");
  private NetworkTableEntry targetArea = table.getEntry("ta");
  private NetworkTableEntry skew = table.getEntry("ts");
  private NetworkTableEntry hasTarget = table.getEntry("tv");

  public Limelight() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public double getDistance() {
    return (Constants.heightDifference) /
        tan(toRadians(Constants.mountingAngle +
            verticalOffset.getDouble(-99)));
  }

  public double getHorizontalOffset() {
    return horizontalOffset.getDouble(-99);
  }

  public double getVerticalOffset() {
    return verticalOffset.getDouble(-99);
  }

  public boolean getHasTarget() {
    return hasTarget.getDouble(-99) == 1 ? true : false;
  }

}
