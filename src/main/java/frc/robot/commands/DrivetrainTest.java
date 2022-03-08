// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DrivetrainTest extends CommandBase {
    Drivetrain drivetrain;

    public DrivetrainTest(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Forward", 0);
        SmartDashboard.putNumber("Sideways", 0);
        SmartDashboard.putNumber("Rotation", 0);
    }

    @Override
    public void execute() {
        drivetrain.drive(SmartDashboard.getNumber("Forward", 0),
                SmartDashboard.getNumber("Sideways", 0),
                SmartDashboard.getNumber("Rotation", 0));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
