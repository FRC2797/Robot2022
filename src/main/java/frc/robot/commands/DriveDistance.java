// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

//Code copied and pasted from the limelight documentation
public class DriveDistance extends CommandBase {
    Drivetrain drivetrain;
    Navx navx;
    double distance;
    BangBangController bangController; 

    //distance should be given in feet
    public DriveDistance(double distance, Drivetrain drivetrain, Navx navx) {
        this.drivetrain = drivetrain; 
        this.navx = navx; 
        this.distance = distance;
        bangController = new BangBangController(0.1);
        addRequirements(drivetrain, navx);
    }

    @Override
    public void initialize() {
        navx.reset();
    }

    @Override
    public void execute() {
        //TODO: This needs to be tested. The getRotation is assumed that if it goes back to the original rotation it goes back to zero.
        drivetrain.drive(1, 0, bangController.calculate(navx.getRotation(), 0));
    
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
        navx.reset();
    }

    @Override
    public boolean isFinished() {
        return navx.getDistance() >= distance ? true : false;
    }
}
