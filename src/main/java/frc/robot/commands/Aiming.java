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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//Code copied and pasted from the limelight documentation basically
public class Aiming extends CommandBase {
    private Drivetrain drivetrain;
    private Limelight limelight;
    private PIDController pidController;
    private XboxController xboxController = new XboxController(0);

    public Aiming(Limelight limelight, Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(limelight, drivetrain);
        pidController = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("kP", 0.05);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        
        // Makes it so it won't crash in simulation
        if (limelight == null) {
            System.out.println("ERROR Limelight is null");
            cancel();
        }
        if (!limelight.getHasTarget()) {
            new StartEndCommand(() -> xboxController.setRumble(RumbleType.kRightRumble, 0.5),
                    () -> xboxController.setRumble(RumbleType.kRightRumble, 0)).withTimeout(0.2).schedule();
            cancel();
        }

    }

    @Override
    public void execute() {
        pidController.setP(SmartDashboard.getNumber("kP", 0));
        pidController.setI(SmartDashboard.getNumber("kI", 0));
        pidController.setD(SmartDashboard.getNumber("kD", 0));
        drivetrain.drive(0, 0, -pidController.calculate(limelight.getHorizontalOffset()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
