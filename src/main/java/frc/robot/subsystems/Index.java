// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Motors.Index;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(Index, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder(); 

  public Index() {
    encoder.setPositionConversionFactor(Constants.encoderConstantsIndex.outputRotationInAInputRotation); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("index encoder", getOutputRotations());
  }

  @Override
  public void simulationPeriodic() {
  }

  public void on() {
    motor.set(Constants.indexPower);
  }

  public void slowOn() {
    motor.set(0.1);
  }

  public void off() {
    motor.set(0);
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public double getOutputRotations() {
    return encoder.getPosition(); 
  }

}
