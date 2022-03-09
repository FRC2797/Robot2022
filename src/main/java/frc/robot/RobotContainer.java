// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  // Trigger intializations
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

  final private JoystickButton leftStickClicked = new JoystickButton(xboxController,
      XboxController.Button.kLeftStick.value);

  final private Trigger leftStickUp = new Trigger() {
    public boolean get() {
      return xboxController.getLeftY() <= -Constants.drivingDeadzone;
    };
  };

  final private Trigger leftStickDown = new Trigger() {
    public boolean get() {
      return xboxController.getLeftY() >= Constants.drivingDeadzone;
    };
  };

  final private Trigger rightStickUp = new Trigger() {
    public boolean get() {
      return xboxController.getRightY() <= -Constants.drivingDeadzone;
    };
  };

  final private Trigger rightStickDown = new Trigger() {
    public boolean get() {
      return xboxController.getRightY() >= Constants.drivingDeadzone;
    };
  };

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
  //

  // Command declarations
  final Command teleopDriving;

  final Command shooterAnalog;
  final Command aimRevThenWait;
  final Command drivetrainTest;

  final Command climberFrontUp;
  final Command climberFrontDown;
  final Command climberRearUp;
  final Command climberRearDown;
  final Command aimRevThenWaitWithCondition;

  final Command climberFrontRightUp;
  final Command climberFrontRightDown;
  final Command climberFrontLeftUp;
  final Command climberFrontLeftDown;
  final Command climberRearRightUp;
  final Command climberRearRightDown;
  final Command climberRearLeftUp;
  final Command climberRearLeftDown;
  SendableChooser<Command> chooser;
  //

  public RobotContainer() {
    // Encoder resetting
    drivetrain.resetEncoders();
    index.resetEncoder();
    navx.reset();
    //

    //Sendable chooser for autonomous
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Get data from colin1", getAutonomousCommand(0));
    chooser.addOption("Get data from colin2", getAutonomousCommand(0));
    chooser.addOption("Get data from colin3", getAutonomousCommand(0));
    SmartDashboard.putData(chooser);
    //

    // Command Initilizations
    drivetrainTest = new DrivetrainTest(drivetrain).withName("drivetrainTest");

    shooterAnalog = new FunctionalCommand(() -> {
    }, () -> {
      shooter.setSpeed((xboxController.getRightTriggerAxis()));
    }, interrupt -> shooter.setSpeed(0), () -> {
      return false;
    }, shooter).withName("shooterAnalog");


    //They don't have climber as a requirement so multiple of them can run at the same time
    //yes, this is a terrible way of doing it
    climberFrontUp = new StartEndCommand(climber::setFrontUp, climber::frontOff).withName("climberFrontUp");
    climberFrontDown = new StartEndCommand(climber::setFrontDown, climber::frontOff)
        .withName("climberFrontDown");
    climberRearUp = new StartEndCommand(climber::setRearUp, climber::rearOff).withName("climberRearUp");
    climberRearDown = new StartEndCommand(climber::setRearDown, climber::rearOff).withName("climberRearDown");

    climberFrontRightUp = new StartEndCommand(climber::setFrontRightUp, climber::frontRightOff)
        .withName("climberFrontRightUp");
    climberFrontRightDown = new StartEndCommand(climber::setFrontRightDown, climber::frontRightOff)
        .withName("climberFrontRightDown");

    climberFrontLeftUp = new StartEndCommand(climber::setFrontLeftUp, climber::frontLeftOff)
        .withName("climberFrontLeftUp");
    climberFrontLeftDown = new StartEndCommand(climber::setFrontLeftDown, climber::frontLeftOff)
        .withName("climberFrontLeftDown");

    climberRearRightUp = new StartEndCommand(climber::setRearRightUp, climber::rearRightOff)
        .withName("climberRearRightUp");
    climberRearRightDown = new StartEndCommand(climber::setRearRightDown, climber::rearRightOff)
        .withName("climberRearRightDown");

    climberRearLeftUp = new StartEndCommand(climber::setRearLeftUp, climber::rearLeftOff)
        .withName("climberRearLeftUp");
    climberRearLeftDown = new StartEndCommand(climber::setRearLeftDown, climber::rearLeftOff)
        .withName("climberRearLeftDown");

    aimRevThenWait = new SequentialCommandGroup(
        new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
        new ParallelRaceGroup(new shooterRevLimelightDistance(shooter, limelight),
            new WaitUntilPeakShooterRPM(shooter)
                .andThen(xboxControllerStartEndRumbleCommand(RumbleType.kRightRumble, 0.1, 9999, "Waiting for index"))))
                    .withName("aimRevThenWait");

    aimRevThenWaitWithCondition = new ConditionalCommand(aimRevThenWait,
        xboxControllerStartEndRumbleCommand(RumbleType.kRightRumble, 0.2, 0.1, "Error Rumble aimRevThenWait"),
        () -> (!navx.isConnected()) && (!limelight.getHasTarget()))
            .withName("aimShootThenIndexWithCondition");
    //

    // Scheme switching
    startButt.whenPressed(() -> {
      CommandScheduler.getInstance().cancelAll();
      if (isSemiAuto.get()) {
        currentScheme = Scheme.manual.value;
      } else if (isManual.get()) {
        currentScheme = Scheme.semiAuto.value;
      } else if (isClimber.get()) {
        currentScheme = Scheme.semiAuto.value;
      }
    });

    backButt.whenPressed(() -> {
      CommandScheduler.getInstance().cancelAll();
      currentScheme = Scheme.climber.value;
    });

    // Semi-autonomous
    lTrig.and(isSemiAuto).and(bButt.negate()).whileActiveOnce(intakeInOnOff());
    lTrig.and(bButt).and(isSemiAuto)
        .whileActiveOnce(new ParallelCommandGroup(intakeOutOnOff(), indexOutOnOff()));

    // Intention is that the user can aimRevThenWait whenever they want, they then
    // use the right bumper to fire off what ever balls
    // they have. The user then manually ends aimRevThenWait again
    rTrig.and(isSemiAuto).toggleWhenActive(aimRevThenWaitWithCondition);
    rBump.and(isSemiAuto).toggleWhenActive(new IndexRevolve(Constants.indexFromIntakeRevolutions, index).andThen(
        xboxControllerStartEndRumbleCommand(RumbleType.kLeftRumble, 0.5, 0.1, "index from intake done rumble")));

    // Manual
    lTrig.and(isManual).and(bButt.negate()).whileActiveOnce(intakeInOnOff());
    lTrig.and(isManual).and(bButt).whileActiveOnce(intakeOutOnOff());
    rBump.and(isManual).and(bButt.negate()).whileActiveOnce(indexInOnOff());
    rBump.and(isManual).and(bButt).whileActiveOnce(indexOutOnOff());
    rTrig.and(isManual).whileActiveContinuous(shooterAnalog);

    // Climbing
    leftStickUp.and(isClimber).whileActiveOnce(climberRearLeftUp);
    leftStickDown.and(isClimber).whileActiveOnce(climberRearLeftDown);
    rightStickUp.and(isClimber).whileActiveOnce(climberRearRightUp);
    rightStickDown.and(isClimber).whileActiveOnce(climberRearRightDown);

    lBump.and(isClimber).whileActiveOnce(climberFrontLeftUp);
    lTrig.and(isClimber).whileActiveOnce(climberFrontLeftDown);
    rBump.and(isClimber).whileActiveOnce(climberFrontRightUp);
    rTrig.and(isClimber).whileActiveOnce(climberFrontRightDown);

    // Driving
    teleopDriving = new RunCommand(() -> {
      if (!isClimber.get()) {
        teleopDrivingFullSpeed();
      } else {
        teleopDrivingClimber();
      }
    }, drivetrain).withName("teleopDriving");

    drivetrain.setDefaultCommand(teleopDriving);

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
    /*
     * The left y is inverted not because the drive method has negative be forward
     * but because the controller returns a negative value
     * forward for left y
     */
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

  public Command xboxControllerStartEndRumbleCommand(RumbleType rType, double intensity, double waitInSeconds,
      String name) {
    return new StartEndCommand(() -> xboxController.setRumble(rType, intensity),
        () -> xboxController.setRumble(rType, 0)).withTimeout(waitInSeconds).withName("xboxControllerRumble");
  }

  public Command getAutonomousCommand(double distanceInInches) {
    //we start with oneball ready to index into the shooter
    return new ParallelCommandGroup(intakeInOnOff(), new SequentialCommandGroup(
          new DriveDistance(distanceInInches, drivetrain),
          new DriveRotation(180, drivetrain, navx, xboxController),
          new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
          new ParallelCommandGroup(new shooterRevLimelightDistance(shooter, limelight),
              new SequentialCommandGroup(new WaitUntilPeakShooterRPM(shooter),
                  new IndexRevolve(Constants.indexFromIntakeRevolutions, index),
                  new WaitUntilPeakShooterRPM(shooter),
                  new IndexRevolve(Constants.indexFromIntakeRevolutions, index)))));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public void putSmartDashboardValues() {
    SmartDashboard.putString("Current mode", Scheme.stringFromValue(currentScheme));
    SmartDashboard.putBoolean("Manual", isManual.get());
    SmartDashboard.putBoolean("Semi Auto", isSemiAuto.get());
    SmartDashboard.putBoolean("Climber", isClimber.get());

    SmartDashboard.putBoolean("Has Target", limelight.getHasTarget());
    SmartDashboard.putNumber("LL Distance", limelight.getDistance());
  }
}