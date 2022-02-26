// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  private boolean isManualBool = true;

  final private XboxController xboxController = new XboxController(0);

  final private Trigger isManual = new Trigger() {
    public boolean get() {
      return isManualBool;
    }
  };

  final private JoystickButton rBump = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
  final private Trigger rBumpManual = rBump.and(isManual);
  final private Trigger rBumpSemiAuto = rBump.and(isManual.negate());

  final private JoystickButton lBump = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
  final private Trigger lBumpManual = lBump.and(isManual);
  final private Trigger lBumpSemiAuto = lBump.and(isManual.negate());

  final private JoystickButton bButt = new JoystickButton(xboxController, XboxController.Button.kB.value);
  final private Trigger bButtManual = bButt.and(isManual);
  final private Trigger bButtSemiAuto = bButt.and(isManual.negate());

  final private JoystickButton aButt = new JoystickButton(xboxController, XboxController.Button.kA.value);
  final private Trigger aButtManual = aButt.and(isManual);
  final private Trigger aButtSemiAuto = aButt.and(isManual.negate());

  final private JoystickButton yButt = new JoystickButton(xboxController, XboxController.Button.kY.value);
  final private Trigger yButtManual = yButt.and(isManual);
  final private Trigger yButtSemiAuto = yButt.and(isManual.negate());

  final private JoystickButton xButt = new JoystickButton(xboxController, XboxController.Button.kX.value);
  final private Trigger xButtManual = xButt.and(isManual);
  final private Trigger xButtSemiAuto = xButt.and(isManual.negate());

  final private JoystickButton startButt = new JoystickButton(xboxController, XboxController.Button.kStart.value);
  // Back buttons is used for testing commands
  final private JoystickButton backButt = new JoystickButton(xboxController, XboxController.Button.kBack.value);

  final private Trigger dpadUp = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 0 ? true : false;
    };
  };
  final private Trigger dpadUpManual = dpadUp.and(isManual);
  final private Trigger dpadUpSemiAuto = dpadUp.and(isManual.negate());

  final private Trigger dpadRight = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 90 ? true : false;
    };
  };
  final private Trigger dpadRightManual = dpadRight.and(isManual);
  final private Trigger dpadRightSemiAuto = dpadRight.and(isManual.negate());

  final private Trigger dpadDown = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 180 ? true : false;
    };
  };
  final private Trigger dpadDownManual = dpadDown.and(isManual);
  final private Trigger dpadDownSemiAuto = dpadDown.and(isManual.negate());

  final private Trigger dpadLeft = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 270 ? true : false;
    };
  };
  final private Trigger dpadLeftManual = dpadLeft.and(isManual);
  final private Trigger dpadLeftSemiAuto = dpadLeft.and(isManual.negate());

  final private Trigger rTrig = new Trigger() {
    public boolean get() {
      return xboxController.getRightTriggerAxis() > Constants.triggerDeadzone ? true : false;
    };
  };
  final private Trigger rTrigManual = rTrig.and(isManual);
  final private Trigger rTrigSemiAuto = rTrig.and(isManual.negate());

  final private Trigger lTrig = new Trigger() {
    public boolean get() {
      return xboxController.getLeftTriggerAxis() > Constants.triggerDeadzone ? true : false;
    };
  };
  final private Trigger lTrigManual = lTrig.and(isManual);
  final private Trigger lTrigSemiAuto = lTrig.and(isManual.negate());

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
          SmartDashboard.putBoolean("isManual", isManualBool);
        }, limelight).withName("ll SmartDashboard.put() values"));

    // DRIVING
    // TODO: Add deadzone
    Command teleopDriving = new RunCommand(
        () -> {
          drivetrain.drive(
              inputFilter(xboxController.getLeftY()),
              inputFilter(xboxController.getLeftX()),
              inputFilter(xboxController.getRightX()));
        }, drivetrain).withName("teleopDriving");


    Command drivetrainTest = new DrivetrainTest(drivetrain).withName("drivetrainTest"); 
    drivetrain.setDefaultCommand(teleopDriving);

    configureButtonBindings();

  }

  public void configureButtonBindings() {
    Command shooterAnalog = new FunctionalCommand(() -> {
    }, () -> {
      shooter.setSpeed((xboxController.getRightTriggerAxis()));
    }, interrupt -> shooter.setSpeed(0), () -> {
      return false;
    }, shooter).withName("shooterAnalog"); 

    Command shooterRevLimelightDistance = new StartEndCommand(() -> shooter.setSpeedDistance(limelight.getDistance()),
        () -> shooter.setSpeed(0), shooter, limelight).withName("shooterRevLimelightDistance");

    // The wait command is so that the interrupt boolean isn't checked before reset
    // encoder is run
    // TODO: Needs testing
    Command indexOnceFromIntake = indexRevolve(Constants.indexFromIntakeRevolutions, "indexOnceFromIntake");
    
    Command indexIntoShooter = indexRevolve(Constants.indexIntoShooterRevolutions, "indexIntoShooter");

    Command intakeOnOff = new StartEndCommand(intake::on, intake::off, intake).withName("intakeOnOff");
    Command indexOnOff = new StartEndCommand(index::on, index::off, index).withName("indexOnOff");

    Command aimShootThenIndex = new SequentialCommandGroup(new Aiming(limelight,
    drivetrain, shooter), new ParallelRaceGroup(shooterRevLimelightDistance, new WaitCommand(0.1).andThen(indexIntoShooter))).withName("aimShootThenIndex");



    startButt.whenPressed(() -> {
      isManualBool = isManualBool ? false : true;
      CommandScheduler.getInstance().cancelAll();
    });

    // Semi-autonomous
    lTrigSemiAuto.whileActiveContinuous(intakeOnOff);
    //TODO: Need to make aimShootThenIndex stop when released
    rTrigSemiAuto.whileActiveOnce(aimShootThenIndex);
    rBumpSemiAuto.whileActiveOnce(indexOnceFromIntake);

    // Manual
    lTrigManual.whileActiveContinuous(intakeOnOff);
    rTrigManual.whileActiveContinuous(shooterAnalog);
    rBumpManual.whileActiveContinuous(indexOnOff);

    // testing
    backButt.whileActiveContinuous(new Aiming(limelight, drivetrain, shooter));

  }

  public double inputFilter(double input) {
    return input <= Constants.drivingDeadzone && input >= -Constants.drivingDeadzone ? 0 : input;
  }

  public void displayControllerSticks() {
    SmartDashboard.putNumber("Left X", xboxController.getLeftX());
    SmartDashboard.putNumber("Left Y", xboxController.getLeftY());
    SmartDashboard.putNumber("Right X", xboxController.getRightX());
    SmartDashboard.putNumber("Right Y", xboxController.getRightY());
  }

  private Command indexRevolve(double revolutions, String name) {
    return new InstantCommand(() -> index.resetEncoder(), index).andThen(new WaitCommand(0))
        .andThen(
            new StartEndCommand(index::on, index::off, index).withInterrupt(() -> index.getOutputRotations() >= revolutions)).withName(name);
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