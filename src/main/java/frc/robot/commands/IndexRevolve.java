
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;

public class IndexRevolve extends CommandBase {
  boolean doneOnce = false;
  Index index;
  PIDController pidController = new PIDController(Constants.indexKp, 0, 0);

  public IndexRevolve(double revolutions, Index index) {
    withName("IndexRevolve" + " " + revolutions);
    this.index = index;
    this.pidController.setSetpoint(revolutions);
    this.pidController.setTolerance(0.05);
    addRequirements(index);
  }

  @Override
  public void initialize() {
    index.resetEncoder();
  }

  @Override
  public void execute() {
    index.setSpeed(pidController.calculate(index.getOutputRotations()));
  }

  @Override
  public void end(boolean interrupted) {
    index.off();
    index.resetEncoder();
  }

  @Override
  public boolean isFinished() {
    //Need doneonce because reset encoders doesn't reset fast enough
    if (doneOnce) {
      doneOnce = false;
      return false;
    }
    return pidController.atSetpoint();
  }
}
