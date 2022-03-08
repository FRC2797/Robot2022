// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.shooterRevLimelightDistance;
import frc.robot.commands.WaitUntilPeakShooterRPM;
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

  Command teleopDriving;
  Command shooterAnalog;
  Command shooterRevLimelightDistance;
  Command indexFromIntake;
  Command indexIntoShooter;
  Command aimShootThenIndex;
  Command drivetrainTest;

  Command autoIntakeInOnOff;
  Command autoIntakeOutOnOff;
  Command autoIndexInOnOff;
  Command autoIndexOutOnOff;

  Command controllerIntakeInOnOff;
  Command controllerIntakeOutOnOff;
  Command controllerIndexInOnOff;
  Command controllerIndexOutOnOff;
  
  Command climberFrontUp;
  Command climberFrontDown;
  Command climberRearUp;
  Command climberRearDown;
  Command xboxControllerRumble;
  Command aimShootThenIndexWithCondition;
  Command waitUntilPeakShooterRPM;

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
    teleopDriving = new RunCommand(
        () -> {
          drivetrain.drive(
              inputFilter(-xboxController.getLeftY()),
              inputFilter(xboxController.getLeftX()),
              inputFilter(xboxController.getRightX()));
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

    aimShootThenIndex = new SequentialCommandGroup(new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
        new ParallelRaceGroup(shooterRevLimelightDistance,
            waitUntilPeakShooterRPM.andThen(indexIntoShooter)))
                .withName("aimShootThenIndex");
    xboxControllerRumble = new StartEndCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.2),
        () -> xboxController.setRumble(RumbleType.kLeftRumble, 0)).withTimeout(0.1).withName("xboxControllerRumble");
        
    aimShootThenIndexWithCondition = new ConditionalCommand(aimShootThenIndex, xboxControllerRumble,
        () -> limelight.getHorizontalOffset() != -99).withName("aimShootThenIndexWithCondition");

    drivetrainTest = new DrivetrainTest(drivetrain).withName("drivetrainTest");

    limelight.setDefaultCommand(
        new RunCommand(() -> {
          SmartDashboard.putNumber("Distance", limelight.getDistance());
          SmartDashboard.putBoolean("Has Target", limelight.getHasTarget());
          SmartDashboard.putBoolean("isManual", isManualBool);
        }, limelight).withName("ll SmartDashboard.put() values"));

    drivetrain.setDefaultCommand(teleopDriving);

    startButt.whenPressed(() -> {
      isManualBool = isManualBool ? false : true;
      CommandScheduler.getInstance().cancelAll();
    });

    // Semi-autonomous
    lTrigSemiAuto.and(bButtSemiAuto.negate()).whileActiveOnce(controllerIntakeInOnOff);
    lTrigSemiAuto.and(bButtSemiAuto).whileActiveOnce(new ParallelCommandGroup(controllerIntakeOutOnOff, controllerIndexOutOnOff));
    rTrigSemiAuto.toggleWhenActive(aimShootThenIndex);
    rBumpSemiAuto.toggleWhenActive(indexFromIntake);

    dpadUpSemiAuto.whileActiveOnce(climberFrontUp, true);
    dpadDownSemiAuto.whileActiveOnce(climberFrontDown, true);
    dpadLeftSemiAuto.whileActiveOnce(climberRearDown, true);
    dpadRightSemiAuto.whileActiveOnce(climberRearUp, true);

    // Manual
    lTrigManual.and(bButtManual.negate()).whileActiveOnce(controllerIntakeInOnOff);
    lTrigManual.and(bButtManual).whileActiveOnce(controllerIntakeOutOnOff);
    rBumpManual.and(bButtManual.negate()).whileActiveOnce(controllerIndexInOnOff);
    rBumpManual.and(bButtManual).whileActiveOnce(controllerIndexOutOnOff);
    rTrigManual.whileActiveContinuous(shooterAnalog);

    dpadUpManual.whileActiveOnce(climberFrontUp, true);
    dpadDownManual.whileActiveOnce(climberFrontDown, true);
    dpadLeftManual.whileActiveOnce(climberRearDown, true);
    dpadRightManual.whileActiveOnce(climberRearUp, true);

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
        new DriveRotation(180, drivetrain, navx, xboxController), new DriveRotation(limelight.getHorizontalOffset(), drivetrain, navx, xboxController),
        new ParallelCommandGroup(shooterRevLimelightDistance,
            new SequentialCommandGroup(waitUntilPeakShooterRPM, indexIntoShooter,
                waitUntilPeakShooterRPM, indexFromIntake, indexIntoShooter))));
  }
}