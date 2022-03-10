// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Motors.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Intake, MotorType.kBrushless);

  public Intake() {
    intakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
  }

  public void onIn() {
    intakeMotor.set(1);
  }

  public void onOut() {
    intakeMotor.set(-1);
  }


  public void off() {
    intakeMotor.set(0);
  }
}
