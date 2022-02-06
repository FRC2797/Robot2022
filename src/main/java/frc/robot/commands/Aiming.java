// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//Code copied and pasted from the limelight documentation
public class Aiming extends CommandBase {
    Drivetrain drivetrain;
    Limelight limelight;
    Shooter shooter;
    double headingError;
    double steeringAdjust;
    

    
    public Aiming(Limelight limelight, Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shooter = shooter; 
        addRequirements(limelight, drivetrain, shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        headingError = limelight.getHorizontalOffset();
        // if (headingError > 1.0) {
        //     steeringAdjust = Constants.aimingkP 
        // } 
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
