// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Motors.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(kIndex, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder(); 

  public Index() {
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
  }

  public void on() {
    motor.set(-0.6);
  }

  public void off() {
    motor.set(0);
  }

  public double getEncoderCount() {
    //42 counts per revolution
    return encoder.getPosition(); 
  }

  public double getRevolutions() {
    return encoder.getPosition() / encoder.getCountsPerRevolution(); 
  }

}
