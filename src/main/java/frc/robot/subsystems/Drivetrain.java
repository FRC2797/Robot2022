// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Motors.Drivetrain.*;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  final private CANSparkMax frontLeft = new CANSparkMax(FrontLeft, MotorType.kBrushless);
  final private CANSparkMax frontRight = new CANSparkMax(FrontRight, MotorType.kBrushless);
  final private CANSparkMax rearLeft = new CANSparkMax(RearLeft, MotorType.kBrushless);
  final private CANSparkMax rearRight = new CANSparkMax(RearRight, MotorType.kBrushless);

  final private RelativeEncoder frontLeftEnc = frontLeft.getEncoder();
  final private RelativeEncoder frontRightEnc = frontRight.getEncoder();
  final private RelativeEncoder rearLeftEnc = rearLeft.getEncoder();
  final private RelativeEncoder rearRightEnc = rearRight.getEncoder();

  final private MecanumDrive mecanumDrive;

  public Drivetrain() {
    rearRight.setInverted(true);
    frontRight.setInverted(true);

    mecanumDrive = new MecanumDrive(
        frontLeft,
        rearLeft,
        frontRight,
        rearRight);

    // Deadband is zero so that it doesn't affect any autonomous code
    // Add deadzone to the inputs themselves
    mecanumDrive.setDeadband(Constants.deadband);

    frontLeftEnc.setPositionConversionFactor(Constants.encoderConstantsDrivetrain.outputRotationInAInputRotation);
    frontRightEnc.setPositionConversionFactor(Constants.encoderConstantsDrivetrain.outputRotationInAInputRotation);
    rearLeftEnc.setPositionConversionFactor(Constants.encoderConstantsDrivetrain.outputRotationInAInputRotation);
    rearRightEnc.setPositionConversionFactor(Constants.encoderConstantsDrivetrain.outputRotationInAInputRotation);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void drive(double forwardSpeed, double sidewaysSpeed, double rotation) {
    mecanumDrive.driveCartesian(forwardSpeed, sidewaysSpeed, rotation);
  }

  public void resetEncoders() {
    frontLeftEnc.setPosition(0);
    frontRightEnc.setPosition(0);
    rearLeftEnc.setPosition(0);
    rearRightEnc.setPosition(0);
  }

  public double getDistanceDrivenInInches() {
    return Constants.encoderConstantsDrivetrain.wheelCircumference * getWheelRotation();
  }

  public double getWheelRotation() {
    return (frontLeftEnc.getPosition() + frontRightEnc.getPosition() + rearLeftEnc.getPosition()
        + rearRightEnc.getPosition()) / 4;
  }

  public void setIdleModetoBrake() {
    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    rearLeft.setIdleMode(IdleMode.kBrake);
    rearRight.setIdleMode(IdleMode.kBrake);
  }

  public void setIdleModeToCoast() {
    frontLeft.setIdleMode(IdleMode.kCoast);
    frontRight.setIdleMode(IdleMode.kCoast);
    rearLeft.setIdleMode(IdleMode.kCoast);
    rearRight.setIdleMode(IdleMode.kCoast);
  }
}
