// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;

//TODO: Needs testing
public class DriveRotation extends CommandBase {
    Drivetrain drivetrain;
    Navx navx;
    double rotation;
    double initialRotation;
    boolean isClockwise;
    PIDController pidController;
    XboxController xboxController;
    double kP = Constants.driveRotationkP;
    double kI = Constants.driveRotationkI;
    double kD = Constants.driveRotationkD;

    // distance should be given in feet
    public DriveRotation(double rotation, Drivetrain drivetrain, Navx navx, XboxController xboxController) {

        withName("rotate " + rotation);
        this.drivetrain = drivetrain;
        this.navx = navx;
        this.rotation = rotation;
        this.xboxController = xboxController;
        this.pidController = new PIDController(kP, kI, kD);
        this.pidController.setSetpoint(rotation);
        withName("rotate " + rotation); 
        
        this.pidController.setTolerance(0);
        addRequirements(drivetrain, navx);

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
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Driverotation navx rotation", navx.getRotation());
        drivetrain.drive(0, 0, pidController.calculate(navx.getRotation()));
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
