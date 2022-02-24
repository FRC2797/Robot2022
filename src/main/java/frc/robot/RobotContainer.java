// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aiming;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DrivetrainTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
  final private Drivetrain drivetrain = new Drivetrain();
  final private Shooter shooter = new Shooter();
  final private Intake intake = new Intake();
  final private Limelight limelight = new Limelight();
  final private Index index = new Index();
  final private Navx navx = new Navx();

  final private XboxController xboxController = new XboxController(0);

  final private JoystickButton rBump = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
  final private JoystickButton lBump = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
  final private JoystickButton bButt = new JoystickButton(xboxController, XboxController.Button.kB.value);
  final private JoystickButton aButt = new JoystickButton(xboxController, XboxController.Button.kA.value);
  final private JoystickButton yButt = new JoystickButton(xboxController, XboxController.Button.kY.value);
  final private JoystickButton xButt = new JoystickButton(xboxController, XboxController.Button.kX.value);

  final private Trigger rTrig = new Trigger() {
    public boolean get() {
      if (xboxController.getRightTriggerAxis() > Constants.triggerDeadzone) {
        return true;
      } else {
        return false;
      }
    };
  };
  final private Trigger lTrig = new Trigger() {
    public boolean get() {
      if (xboxController.getLeftTriggerAxis() > Constants.triggerDeadzone) {
        return true;
      } else {
        return false;
      }
    };
  };

  // Command
  public RobotContainer() {
    drivetrain.resetEncoders();
    index.resetEncoder();
    navx.reset();

    limelight.setDefaultCommand(
        new RunCommand(() -> {
          SmartDashboard.putNumber("Distance", limelight.getDistance());
          SmartDashboard.putBoolean("Has Target", limelight.getHasTarget());
          SmartDashboard.putNumber("vertical", limelight.getVerticalOffset());
        }, limelight));

    // DRIVING
    // TODO: Add deadzone
    RunCommand teleopDriving = new RunCommand(
        () -> {
          drivetrain.drive(
              inputFilter(xboxController.getLeftY()),
              inputFilter(xboxController.getLeftX()),
              inputFilter(xboxController.getRightX()));
        }, drivetrain);

    DrivetrainTest drivetrainTest = new DrivetrainTest(drivetrain);
    drivetrain.setDefaultCommand(teleopDriving);

    configureButtonBindings();
  }

  // TODO: make the scheme switchable, maybe only use triggers and have them look
  // at a boolean?
  // Conditional commands?
  public void configureButtonBindings() {

    Command shooterOnOff = new FunctionalCommand(() -> {
    }, () -> {
      shooter.setSpeed((xboxController.getRightTriggerAxis()));
    }, interrupt -> shooter.setSpeed(0), () -> {
      return false;
    }, shooter);

    // The wait command is so that the interrupt boolean isn't checked before reset
    // encoder is run
    // TODO: Needs testing
    Command indexOnce = new InstantCommand(() -> index.resetEncoder(), index).andThen(new WaitCommand(0)).andThen(
        new StartEndCommand(index::on, index::off, index).withInterrupt(() -> index.getOutputRotations() >= 2.25));

    Command intakeOnOff = new StartEndCommand(intake::on, intake::off, intake);
    Command indexOnOff = new StartEndCommand(index::on, index::off, index);

    lBump.whenPressed(() -> intakeOnOff.schedule()).whenReleased(() -> intakeOnOff.cancel());
    yButt.whenPressed(new Aiming(limelight, drivetrain, shooter));
    rBump.whenPressed(() -> indexOnce.schedule());
    rTrig.whileActiveOnce(new ScheduleCommand(shooterOnOff))
        .whenInactive(new InstantCommand(() -> shooterOnOff.cancel()));

  }

  public double inputFilter(double input) {
    // TODO: Add deadzone to constants
    return input <= 0.2 && input >= -0.2 ? 0 : input;
  }

  public void displayControllerSticks() {
    SmartDashboard.putNumber("Left X", xboxController.getLeftX());
    SmartDashboard.putNumber("Left Y", xboxController.getLeftY());
    SmartDashboard.putNumber("Right X", xboxController.getRightX());
    SmartDashboard.putNumber("Right Y", xboxController.getRightY());
  }

  public Command getAutonomousCommand() {
    Command intakeOn = new RunCommand(intake::on, intake);

    return new SequentialCommandGroup(
        new ScheduleCommand(intakeOn),
        new DriveDistance(Constants.driveDistance, drivetrain, navx),
        new DriveRotation(180, true, drivetrain, navx),
        new Aiming(limelight, drivetrain, shooter),
        new ScheduleCommand(
            new RunCommand(() -> shooter.setSpeedDistance(limelight.getDistance()), shooter, limelight)),
        new WaitCommand(Constants.spinUpTime),
        new StartEndCommand(index::on, index::off, index).withTimeout(Constants.indexWaitTime),
        new WaitCommand(Constants.spinUpTime / 3),
        new StartEndCommand(index::on, index::off, index).withTimeout(Constants.indexWaitTime));

  }
}