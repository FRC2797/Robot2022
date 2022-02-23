// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

//TODO: Needs testing
public class DriveRotation extends CommandBase {
    Drivetrain drivetrain;
    Navx navx;
    double rotation;
    double initialRotation;
    boolean isClockwise; 

    // distance should be given in feet
    public DriveRotation(double rotation, boolean isClockwise, Drivetrain drivetrain, Navx navx) {
        this.drivetrain = drivetrain;
        this.navx = navx;
        this.rotation = rotation;
        this.isClockwise = isClockwise; 
        addRequirements(drivetrain, navx);
    }

    @Override
    public void initialize() {
        navx.reset();
    }

    @Override
    public void execute() {
        drivetrain.drive(0, 0, isClockwise ? 1 : -1);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
        navx.reset();
    }

    @Override
    public boolean isFinished() {
        return navx.getRotation() >= rotation ? true : false;
    }
}
