// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    Drivetrain drivetrain;
    double distance;
    double kP = Constants.driveDistancekP;
    double minTerm = 0.1;
    PIDController pidController = new PIDController(kP, 0, 0);
    boolean encodersReset = false;


    //distance should be given in inches
    public DriveDistance(double distance, Drivetrain drivetrain) {
        this.drivetrain = drivetrain; 
        this.distance = distance;
        this.pidController.setSetpoint(distance);
        this.pidController.setTolerance(8);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
      drivetrain.resetEncoders();
    }

    @Override
    public void execute() {     
      double calculate = pidController.calculate(drivetrain.getDistanceDrivenInInches());
        if (Math.abs(calculate) > minTerm) {
            drivetrain.drive(calculate, 0, 0);
          } else {
            if (calculate > 0) {
              drivetrain.drive(minTerm, 0, 0);
            }
            if (calculate < 0) {
              drivetrain.drive(-minTerm, 0, 0);
            }
          }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
        drivetrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getDistanceDrivenInInches() >= distance ? true : false;
    }
}
