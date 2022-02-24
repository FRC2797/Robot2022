// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

//TODO: Needs testing
public class DriveDistance extends CommandBase {
    Drivetrain drivetrain;
    Navx navx;
    double rotation;
    double distance;
    //distance should be given in feet
    public DriveDistance(double distance, Drivetrain drivetrain, Navx navx) {
        this.drivetrain = drivetrain; 
        this.navx = navx; 
        this.distance = distance;
        addRequirements(drivetrain, navx);
    }

    @Override
    public void initialize() {
        navx.reset();
    }

    @Override
    public void execute() {
        /* TODO: - The getRotation is assumed that if it goes back to the original rotation it goes back to zero. 
                 - Might need a really low Proportional term
        */
        
        double error = navx.getRotation();
        //FIXME: the rotation autocorrecting isn't working
        drivetrain.drive(1, 0, 0);
    
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
        navx.reset();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getDistanceDriven() >= distance ? true : false;
    }
}
