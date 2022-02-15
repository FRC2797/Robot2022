// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Motors.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftMotor = new CANSparkMax(kShooter1, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(kShooter2, MotorType.kBrushless);
  private MotorControllerGroup flyWheelMotors = new MotorControllerGroup(leftMotor, rightMotor);

  public Shooter() {
    leftMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left Motor encoder", leftMotor.getEncoder().getVelocity()); 
    SmartDashboard.putNumber("right motor encoder", rightMotor.getEncoder().getVelocity()); 
    
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Sim Shooter Values", flyWheelMotors.get()); 
  }

  public void shoot(double distance) {

  }

 public void setSpeed(double speed) {
    flyWheelMotors.set(speed);
  } 

  public void fullSpeed() {
    setSpeed(1);
  }

  public void off() {
    setSpeed(0);
  }

}
