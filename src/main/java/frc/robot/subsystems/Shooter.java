// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Motors.leftShooter;
import static frc.robot.Constants.Motors.rightShooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftMotor = new CANSparkMax(leftShooter, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(rightShooter, MotorType.kBrushless);
  private RelativeEncoder leftEnc = leftMotor.getEncoder();
  private RelativeEncoder rightEnc = rightMotor.getEncoder();

  private MotorControllerGroup flyWheelMotors = new MotorControllerGroup(leftMotor, rightMotor);

  public Shooter() {
    leftMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
  }

 public void setSpeed(double speed) {
    flyWheelMotors.set(-speed);
  } 

  public void fullSpeed() {
    setSpeed(1);
  }

  public void off() {
    setSpeed(0);
  }

  public double getAverageRPM() {
    return (getLeftRPM() + getRightRPM()) / 2; 
  }

  public double getLeftRPM() {
    return leftEnc.getVelocity();
  }

  public double getRightRPM() {
    return rightEnc.getVelocity();
  }

}
