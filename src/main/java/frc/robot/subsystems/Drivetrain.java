// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Motors.*;



public class Drivetrain extends SubsystemBase {
  
  


  private CANSparkMax frontLeft = new CANSparkMax(kFrontLeft, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(kFrontRight, MotorType.kBrushless);
  private CANSparkMax rearLeft = new CANSparkMax(kRearLeft, MotorType.kBrushless);
  private CANSparkMax rearRight = new CANSparkMax(kRearRight, MotorType.kBrushless);
  
  private RelativeEncoder frontLeftEnc = frontLeft.getEncoder();
  private RelativeEncoder frontRightEnc = frontRight.getEncoder();
  private RelativeEncoder rearLeftEnc = rearLeft.getEncoder();
  private RelativeEncoder rearRightEnc = rearRight.getEncoder();


  //Motors for old bot
  // private WPI_TalonFX frontLeft = new WPI_TalonFX(2);
  // private WPI_TalonFX frontRight = new WPI_TalonFX(3);
  // private WPI_TalonFX rearLeft = new WPI_TalonFX(1);
  // private WPI_TalonFX rearRight = new WPI_TalonFX(4);
  
  private final MecanumDrive drivetrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);


  
  private MecanumDrive mecanumDrive = new MecanumDrive(
    frontLeft,
    rearLeft,
    frontRight,
    rearRight
  );

  

  public Drivetrain() {
    mecanumDrive.setDeadband(Constants.kdeadband);
    rearRight.setInverted(true);
    frontRight.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front left encoder", frontLeftEnc.getVelocity());
    SmartDashboard.putNumber("Front right encoder", frontRightEnc.getVelocity());
    SmartDashboard.putNumber("rear left encoder", rearLeftEnc.getVelocity());
    SmartDashboard.putNumber("rear right encoder", rearRightEnc.getVelocity());
  }
  
  @Override
  public void simulationPeriodic() {
  }
  
  public void drive(double forwardSpeed, double sidewaysSpeed, double rotation) {
    mecanumDrive.driveCartesian(forwardSpeed, sidewaysSpeed, rotation);
    // mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }
  
   
}
