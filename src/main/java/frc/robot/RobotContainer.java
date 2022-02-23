// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  final public Drivetrain drivetrain = new Drivetrain();
  final private Shooter shooter = new Shooter();
  final private Intake intake = new Intake();
  final private Limelight limelight = new Limelight();
  final private Index index = new Index();
  final private Navx navx = new Navx();

  final public XboxController xboxController = new XboxController(0);

  final private JoystickButton rBump = new JoystickButton(xboxController, Button.kRightBumper.value);
  final private JoystickButton lBump = new JoystickButton(xboxController, Button.kLeftBumper.value);
  final private JoystickButton bButt = new JoystickButton(xboxController, Button.kB.value);
  final private JoystickButton aButt = new JoystickButton(xboxController, Button.kA.value);
  final private JoystickButton yButt = new JoystickButton(xboxController, Button.kY.value);
  final private JoystickButton xButt = new JoystickButton(xboxController, Button.kX.value);

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
    resetEncoders();

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
              inputFilter( xboxController.getLeftY()),
              inputFilter(xboxController.getLeftX()),
              inputFilter(xboxController.getRightX()));
        }, drivetrain);

    DrivetrainTest drivetrainTest = new DrivetrainTest(drivetrain);
    drivetrain.setDefaultCommand(teleopDriving);

    configureButtonBindings();
  }

  public void resetEncoders() {
    drivetrain.resetEncoders();
    index.resetEncoder();
    navx.reset();
  }   

  // TODO: make the scheme switchable, maybe only use triggers and have them look at a boolean?
  // Conditional commands?
  public void configureButtonBindings() {

    rTrig
        .whenActive(() -> shooter.setSpeed(Math.min(xboxController.getRightTriggerAxis(), Constants.shooterPowerLimit)))
        .whenInactive(() -> shooter.setSpeed(0));

    // TODO: Change the intake and index controls to schedule and unschedule a
    // single command
    lBump.whenPressed(new InstantCommand(intake::on, intake))
        .whenReleased(new InstantCommand(intake::off, intake));

    rBump.whileActiveOnce(new InstantCommand(index::on, index))
        .whenInactive(new InstantCommand(index::off, index));

    bButt.whenPressed(new InstantCommand(() -> intake.on(), intake))
        .whenReleased(() -> intake.off(), intake);

    aButt.whenPressed(new StartEndCommand(index::on, index::off, index).withTimeout(Constants.indexWaitTime));


    //FIXME: This doesn't work, the with interrupt reads the getOutputRotations before the encoders are reset, gonna have to add a wait or something
    Command indexOnce = new StartEndCommand(() -> {
      index.resetEncoder();
      index.slowOn();
    }, index::off, index).withInterrupt(() -> index.getOutputRotations() >= 1);
    
    yButt.whenPressed(new DriveDistance(3, drivetrain, navx));

    xButt.whenPressed(new InstantCommand(index::resetEncoder)); 

  }

  public double inputFilter(double input) {
    //TODO: Add deadzone to constants
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