// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitUntilPeakShooterRPM extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  Shooter shooter;
  Timer timer = new Timer();
  double previousRPM = 0;

  public WaitUntilPeakShooterRPM(Shooter shooter) {
    this.shooter = shooter;
    withName("WaitUntilPeakShooterRPM");
    // DO NOT ADD SHOOTER SUBSYSTEM
    addRequirements();
  }

  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    previousRPM = shooter.getAverageRPM();
    DriverStation.reportError("Waiting", false);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    DriverStation.reportError("Done waiting", false);
  }


  @Override
  public boolean isFinished() {
    if (timer.get() >= 2) {
      if ((Math.abs(previousRPM - shooter.getAverageRPM()) / previousRPM) <= 0.02) {
        return true;
      }
      timer.reset();
      previousRPM = shooter.getAverageRPM();
    }
    return false;
  }
}
