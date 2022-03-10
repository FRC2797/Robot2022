
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;

public class IndexRevolve extends CommandBase {
  private boolean doneOnce = false;
  private Index index;
  private PIDController pidController = new PIDController(Constants.indexKp, 0, 0);
  private double minimumTerm = 0.15;


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
    double calculate = pidController.calculate(index.getOutputRotations());
    if (Math.abs(calculate) > minimumTerm) {
      index.setSpeed(calculate);
    } else {
      if (calculate > 0) {
        index.setSpeed(minimumTerm);
      }
      if (calculate < 0) {
        index.setSpeed(-minimumTerm);
      }
    }
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
