// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DrivetrainTest;
import frc.robot.commands.IndexRevolve;
import frc.robot.commands.WaitUntilPeakShooterRPM;
import frc.robot.commands.shooterRevLimelightDistance;
import frc.robot.subsystems.Climber;
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
  final private Climber climber = new Climber();

  public enum Scheme {
    manual(1),
    semiAuto(2),
    climber(3);

    public final int value;

    Scheme(int value) {
      this.value = value;
    }

    public String toString() {
      return name();
    }

    public static String stringFromValue(int value) {
      if (value == Scheme.manual.value) {
        return "manual";
      } else if (value == Scheme.semiAuto.value) {
        return "semiAuto";
      } else if (value == Scheme.climber.value) {
        return "climber";
      }
      return "invalid value";
    }
  }

  private int currentScheme = Scheme.semiAuto.value;

  final private XboxController xboxController = new XboxController(0);

  final private Trigger isManual = new Trigger() {
    public boolean get() {
      return currentScheme == Scheme.manual.value;
    }
  };

  final private Trigger isSemiAuto = new Trigger() {
    public boolean get() {
      return currentScheme == Scheme.semiAuto.value;
    }
  };

  final private Trigger isClimber = new Trigger() {
    public boolean get() {
      return currentScheme == Scheme.climber.value;
    }
  };

  final private JoystickButton rBump = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);

  final private JoystickButton lBump = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);

  final private JoystickButton bButt = new JoystickButton(xboxController, XboxController.Button.kB.value);

  final private JoystickButton aButt = new JoystickButton(xboxController, XboxController.Button.kA.value);

  final private JoystickButton yButt = new JoystickButton(xboxController, XboxController.Button.kY.value);

  final private JoystickButton xButt = new JoystickButton(xboxController, XboxController.Button.kX.value);

  final private JoystickButton startButt = new JoystickButton(xboxController, XboxController.Button.kStart.value);
  // Rear buttons is used for testing commands
  final private JoystickButton backButt = new JoystickButton(xboxController, XboxController.Button.kBack.value);

  final private JoystickButton leftStickDown = new JoystickButton(xboxController,
      XboxController.Button.kLeftStick.value);

  final private Trigger dpadUp = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 0 ? true : false;
    };
  };

  final private Trigger dpadRight = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 90 ? true : false;
    };
  };

  final private Trigger dpadDown = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 180 ? true : false;
    };
  };

  final private Trigger dpadLeft = new Trigger() {
    public boolean get() {
      return xboxController.getPOV() == 270 ? true : false;
    };
  };

  final private Trigger rTrig = new Trigger() {
    public boolean get() {
      return xboxController.getRightTriggerAxis() > Constants.triggerDeadzone ? true : false;
    };
  };

  final private Trigger lTrig = new Trigger() {
    public boolean get() {
      return xboxController.getLeftTriggerAxis() > Constants.triggerDeadzone ? true : false;
    };
  };

  final Command teleopDriving;

  final Command shooterAnalog;
  final Command shooterRevLimelightDistance;
  final Command indexFromIntake;
  final Command indexIntoShooter;
  final Command aimShootThenIndex;
  final Command drivetrainTest;

  final Command autoIntakeInOnOff;
  final Command autoIntakeOutOnOff;
  final Command autoIndexInOnOff;
  final Command autoIndexOutOnOff;

  final Command controllerIntakeInOnOff;
  final Command controllerIntakeOutOnOff;
  final Command controllerIndexInOnOff;
  final Command controllerIndexOutOnOff;

  final Command climberFrontUp;
  final Command climberFrontDown;
  final Command climberRearUp;
  final Command climberRearDown;
  final Command xboxControllerRumble;
  final Command aimShootThenIndexWithCondition;
  final Command waitUntilPeakShooterRPM;

  final Command climberFrontRightUp;
  final Command climberFrontRightDown;
  final Command climberFrontLeftUp;
  final Command climberFrontLeftDown;
  final Command climberRearRightUp;
  final Command climberRearRightDown;
  final Command climberRearLeftUp;
  final Command climberRearLeftDown;

  // Command
  public RobotContainer() {
    drivetrain.resetEncoders();
    index.resetEncoder();
    navx.reset();

    /*
     * The left y is inverted not because the drive method has negative be forward
     * but because the controller returns a negative value
     * forward for left y
     */

    drivetrainTest = new DrivetrainTest(drivetrain).withName("drivetrainTest");

    teleopDriving = new RunCommand(() -> {
      if (!isClimber.get()) {
        teleopDrivingFullSpeed();
      } else {
        teleopDrivingClimber();
      }
    }, drivetrain).withName("teleopDriving");

    shooterAnalog = new FunctionalCommand(() -> {
    }, () -> {
      shooter.setSpeed((xboxController.getRightTriggerAxis()));
    }, interrupt -> shooter.setSpeed(0), () -> {
      return false;
    }, shooter).withName("shooterAnalog");

    shooterRevLimelightDistance = new shooterRevLimelightDistance(shooter, limelight);

    waitUntilPeakShooterRPM = new WaitUntilPeakShooterRPM(shooter);

    indexFromIntake = new IndexRevolve(Constants.indexFromIntakeRevolutions, index);

    indexIntoShooter = new IndexRevolve(Constants.indexIntoShooterRevolutions, index);

    autoIntakeInOnOff = intakeInOnOff();
    autoIntakeOutOnOff = intakeOutOnOff();
    autoIndexInOnOff = indexInOnOff();
    autoIndexOutOnOff = indexOutOnOff();

    controllerIntakeInOnOff = intakeInOnOff();
    controllerIntakeOutOnOff = intakeOutOnOff();
    controllerIndexInOnOff = indexInOnOff();
    controllerIndexOutOnOff = indexOutOnOff();

    climberFrontUp = new StartEndCommand(climber::setFrontUp, climber::frontOff, climber).withName("climberFrontUp");
    climberFrontDown = new StartEndCommand(climber::setFrontDown, climber::frontOff, climber)
        .withName("climberFrontDown");
    climberRearUp = new StartEndCommand(climber::setRearUp, climber::rearOff, climber).withName("climberRearUp");
    climberRearDown = new StartEndCommand(climber::setRearDown, climber::rearOff, climber).withName("climberRearDown");

    climberFrontRightUp = new StartEndCommand(climber::setFrontRightUp, climber::frontRightOff, climber)
        .withName("climberFrontRightUp");
    climberFrontRightDown = new StartEndCommand(climber::setFrontRightDown, climber::frontRightOff, climber)
        .withName("climberFrontRightDown");

    climberFrontLeftUp = new StartEndCommand(climber::setFrontLeftUp, climber::frontLeftOff, climber)
        .withName("climberFrontLeftUp");
    climberFrontLeftDown = new StartEndCommand(climber::setFrontLeftDown, climber::frontLeftOff, climber)
        .withName("climberFrontLeftDown");

    climberRearRightUp = new StartEndCommand(climber::setRearRightUp, climber::rearRightOff, climber)
        .withName("climberRearRightUp");
    climberRearRightDown = new StartEndCommand(climber::setRearRightDown, climber::rearRightOff, climber)
        .withName("climberRearRightDown");

    climberRearLeftUp = new StartEndCommand(climber::setRearLeftUp, climber::rearLeftOff, climber)
        .withName("climberRearLeftUp");
    climberRearLeftDown = new StartEndCommand(climber::setRearLeftDown, climber::rearLeftOff, climber)
        .withName("climberRearLeftDown");

    aimShootThenIndex = new SequentialCommandGroup(
        new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
        new ParallelRaceGroup(shooterRevLimelightDistance,
            waitUntilPeakShooterRPM.andThen(indexIntoShooter)))
                .withName("aimShootThenIndex");
    xboxControllerRumble = new StartEndCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.2),
        () -> xboxController.setRumble(RumbleType.kLeftRumble, 0)).withTimeout(0.1).withName("xboxControllerRumble");

    aimShootThenIndexWithCondition = new ConditionalCommand(aimShootThenIndex, xboxControllerRumble,
        () -> limelight.getHorizontalOffset() != -99).withName("aimShootThenIndexWithCondition");

    limelight.setDefaultCommand(
        new RunCommand(() -> {
          SmartDashboard.putNumber("Distance", limelight.getDistance());
          SmartDashboard.putBoolean("Has Target", limelight.getHasTarget());
          SmartDashboard.putString("Current mode", Scheme.stringFromValue(currentScheme));
        }, limelight).withName("ll SmartDashboard.put() values"));

    drivetrain.setDefaultCommand(teleopDriving);

    // Scheme switching
    startButt.whenPressed(() -> {
      if (isSemiAuto.get()) {
        currentScheme = Scheme.manual.value;
      } else if (isManual.get()) {
        currentScheme = Scheme.semiAuto.value;
      } else if (isClimber.get()) {
        currentScheme = Scheme.semiAuto.value;
      }
    });

    backButt.whenPressed(() -> currentScheme = Scheme.climber.value);

    // Semi-autonomous
    lTrig.and(isSemiAuto).and(bButt.negate()).whileActiveOnce(controllerIntakeInOnOff);
    lTrig.and(bButt).and(isSemiAuto)
        .whileActiveOnce(new ParallelCommandGroup(controllerIntakeOutOnOff, controllerIndexOutOnOff));
    rTrig.and(isSemiAuto).toggleWhenActive(aimShootThenIndex);
    rBump.and(isSemiAuto).toggleWhenActive(indexFromIntake);

    dpadUp.and(isSemiAuto).whileActiveOnce(climberFrontUp, true);
    dpadDown.and(isSemiAuto).whileActiveOnce(climberFrontDown, true);
    dpadLeft.and(isSemiAuto).whileActiveOnce(climberRearDown, true);
    dpadRight.and(isSemiAuto).whileActiveOnce(climberRearUp, true);

    // Manual
    lTrig.and(isManual).and(bButt.negate()).whileActiveOnce(controllerIntakeInOnOff);
    lTrig.and(isManual).and(bButt).whileActiveOnce(controllerIntakeOutOnOff);
    rBump.and(isManual).and(bButt.negate()).whileActiveOnce(controllerIndexInOnOff);
    rBump.and(isManual).and(bButt).whileActiveOnce(controllerIndexOutOnOff);
    rTrig.and(isManual).whileActiveContinuous(shooterAnalog);

    dpadUp.and(isManual).whileActiveOnce(climberFrontUp, true);
    dpadDown.and(isManual).whileActiveOnce(climberFrontDown, true);
    dpadLeft.and(isManual).whileActiveOnce(climberRearDown, true);
    dpadRight.and(isManual).whileActiveOnce(climberRearUp, true);

    // Climbing
    rBump.whileHeld(climberFrontRightUp);
    rTrig.whileActiveContinuous(climberFrontRightDown);

    // testing
    SmartDashboard.putData(new DriveRotation(180, drivetrain, navx, xboxController));

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

  public void teleopDrivingFullSpeed() {
    drivetrain.drive(
        inputFilter(-xboxController.getLeftY()),
        inputFilter(xboxController.getLeftX()),
        inputFilter(xboxController.getRightX()));
  }

  public void teleopDrivingClimber() {
    double forward = 0;
    double sideways = 0;
    double rotation = 0;
    final double SLOW_SPEED = 0.2;

    if (dpadUp.get()) {
      forward += SLOW_SPEED;
    }

    if (dpadDown.get()) {
      forward -= SLOW_SPEED;
    }

    if (dpadRight.get()) {
      sideways += SLOW_SPEED;
    }

    if (dpadLeft.get()) {
      sideways -= SLOW_SPEED;
    }

    drivetrain.drive(forward, sideways, rotation);
  }

  // added to a separate method so it can be reused in multiple places
  public Command intakeInOnOff() {
    return new StartEndCommand(intake::onIn, intake::off, intake).withName("intakeInOnOff");
  }

  public Command intakeOutOnOff() {
    return new StartEndCommand(intake::onOut, intake::off, intake).withName("intakeOutOnOff");
  }

  public Command indexInOnOff() {
    return new StartEndCommand(index::onIn, index::off, index).withName("indexInOnOff");
  }

  public Command indexOutOnOff() {
    return new StartEndCommand(index::onOut, index::off, index).withName("indexOutOnOff");
  }

  public Command getAutonomousCommand() {

    // We start with one ball ready to index into shooter
    return new ParallelCommandGroup(autoIntakeInOnOff, new SequentialCommandGroup(
        new DriveDistance(Constants.autoDriveDistance, drivetrain, navx),
        new DriveRotation(180, drivetrain, navx, xboxController),
        new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
        new ParallelCommandGroup(shooterRevLimelightDistance,
            new SequentialCommandGroup(waitUntilPeakShooterRPM, indexIntoShooter,
                waitUntilPeakShooterRPM, indexFromIntake, indexIntoShooter))));
  }
}