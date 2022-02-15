// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aiming;
import frc.robot.commands.DrivetrainTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  final public Drivetrain drivetrain = new Drivetrain();
  final public XboxController xboxController = new XboxController(0);
  final private Shooter shooter = new Shooter();
  final private Intake intake = new Intake();
  final private Limelight limelight = new Limelight();
  final private Index index = new Index();
  final private Navx navx = new Navx();
  public double waitNumber = SmartDashboard.getNumber("wait", 99);

  final private JoystickButton rBump = new JoystickButton(xboxController, Button.kRightBumper.value);
  final private JoystickButton lBump = new JoystickButton(xboxController, Button.kLeftBumper.value);
  final private JoystickButton bButt = new JoystickButton(xboxController, Button.kB.value);
  final private JoystickButton aButt = new JoystickButton(xboxController, Button.kA.value);
  final private JoystickButton yButt = new JoystickButton(xboxController, Button.kY.value);

  final private Trigger rTrig = new Trigger() {
    public boolean get() {
      if (xboxController.getRightTriggerAxis() > 0.05) {
        return true;
      } else {
        return false;
      }
    };
  };
  final private Trigger lTrig = new Trigger() {
    public boolean get() {
      if (xboxController.getLeftTriggerAxis() > 0.2) {
        return true;
      } else {
        return false;
      }
    };
  };

  public RobotContainer() {
    navx.reset();

    limelight.setDefaultCommand(
        new RunCommand(() -> {
          SmartDashboard.putNumber("Distance", limelight.getDistance());
          SmartDashboard.putBoolean("Has Target", limelight.getHasTarget());
          SmartDashboard.putNumber("vertical", limelight.getVerticalOffset());
        }, limelight));

    // DRIVING
    RunCommand teleopDriving = new RunCommand(
        () -> {
          drivetrain.drive(
              xboxController.getLeftY() * -1,
              xboxController.getLeftX(),
              xboxController.getRightX());
          displayControllerSticks();
        }, drivetrain);

    DrivetrainTest drivetrainTest = new DrivetrainTest(drivetrain);
    drivetrain.setDefaultCommand(drivetrainTest);

    configureButtonBindings();
  }

  // TODO: figure out a way to have switchable schemes
  public void configureButtonBindings() {

    double limit = 0.5;
    rTrig.whenActive(() -> shooter.setSpeed(Math.min(xboxController.getRightTriggerAxis(), limit)))
        .whenInactive(() -> shooter.setSpeed(0));
    lBump.whenPressed(new InstantCommand(intake::on, intake))
        .whenReleased(new InstantCommand(intake::off, intake));

    rBump.whileActiveOnce(new InstantCommand(index::on, index))
        .whenInactive(new InstantCommand(index::off, index));

    bButt.whenPressed(new InstantCommand(() -> intake.on(), intake))
        .whenReleased(() -> intake.off(), intake);

    aButt.whenPressed(
        new SequentialCommandGroup(new InstantCommand(index::on, index),
            new ParallelDeadlineGroup(new WaitCommand(Constants.indexWaitTime), new RunCommand(() -> {
            }, index)),
            new InstantCommand(index::off, index)));

    yButt.whenPressed(new Aiming(limelight, drivetrain, shooter));

    SmartDashboard.putData("reset gyro", new InstantCommand(navx::reset, navx));
  }

  public void displayControllerSticks() {
    SmartDashboard.putNumber("Left X", xboxController.getLeftX());
    SmartDashboard.putNumber("Left Y", xboxController.getLeftY());
    SmartDashboard.putNumber("Right X", xboxController.getRightX());
    SmartDashboard.putNumber("Right Y", xboxController.getRightY());
  }

  // SlewRateLimiter but it doesn't limit decrease
  public double customRateLimit(SlewRateLimiter limiter, double input) {
    double calculation = limiter.calculate(input);
    if (calculation > input) {
      limiter.reset(input);
      return input;
    } else {
      return calculation;
    }
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(); //new SequentialCommandGroup(new InstantCommand(intake::on, intake))
  }
}