// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

//TODO: Needs testing
public class DriveRotation extends CommandBase {
    //TODO: Need to make some values be constants
    Drivetrain drivetrain;
    Navx navx;
    double rotation;
    double initialRotation;
    boolean isClockwise; 
    double kP = 0.003; 
    PIDController pidController;

    // distance should be given in feet
    public DriveRotation(double rotation, Drivetrain drivetrain, Navx navx) {
        this.drivetrain = drivetrain;
        this.navx = navx;
        this.rotation = rotation;
        this.isClockwise = isClockwise; 
        this.pidController = new PIDController(kP, 0, 0);
        this.pidController.setSetpoint(rotation);
        this.pidController.setTolerance(2);
        addRequirements(drivetrain, navx);

    }

    @Override
    public void initialize() {
        navx.reset();
    }

    @Override
    public void execute() {
        double driveRotation; 
        double calculate = pidController.calculate(Math.abs(navx.getRotation()));
        if (calculate >= 0.33) {
            driveRotation = 0.33;
        } else {
            driveRotation = calculate + 0.1;
        }
        
        SmartDashboard.putNumber("Drive rotation for DriveRotation", driveRotation);
        drivetrain.drive(0, 0, driveRotation);

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
