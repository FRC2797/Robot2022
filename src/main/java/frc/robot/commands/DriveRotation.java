// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

public class DriveRotation extends CommandBase {
    Drivetrain drivetrain;
    Navx navx;
    double rotation;
    double initialRotation;
    boolean isClockwise;
    PIDController pidController;
    XboxController xboxController;
    DoubleSupplier doubleSupplier = null; 
    double kP = 0.004;
    double minTerm = 0.07;  

    // distance should be given in feet
    public DriveRotation(double rotation, Drivetrain drivetrain, Navx navx, XboxController xboxController) {

        withName("rotate " + rotation);
        this.drivetrain = drivetrain;
        this.navx = navx;
        this.rotation = rotation;
        this.xboxController = xboxController;
        this.pidController = new PIDController(kP, 0, 0);
        this.pidController.setSetpoint(rotation);
        withName("rotate " + rotation);
        
        this.pidController.setTolerance(0.5);
        addRequirements(drivetrain, navx);
    }

    public DriveRotation(DoubleSupplier doubleSupplier, Drivetrain drivetrain, Navx navx, XboxController xboxController) {
      this(doubleSupplier.getAsDouble(), drivetrain, navx, xboxController);
      this.doubleSupplier = doubleSupplier;
  }

    @Override
    public void initialize() {
        if (navx.isConnected() == false) {
            cancel();
            new StartEndCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.1),
                    () -> xboxController.setRumble(RumbleType.kLeftRumble, 0)).withTimeout(1)
                            .withName("ERROR: NO NAVX");
        }
        navx.reset();
        if (doubleSupplier != null) {
          pidController.setSetpoint(doubleSupplier.getAsDouble());
        }
    }

    @Override
    public void execute() {
        double calculate = pidController.calculate(navx.getRotation());
        if (Math.abs(calculate) > minTerm) {
            drivetrain.drive(0, 0, calculate);
          } else {
            if (calculate > 0) {
              drivetrain.drive(0, 0, minTerm);
            }
            if (calculate < 0) {
              drivetrain.drive(0, 0, -minTerm);
            }
          }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
        navx.reset();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
