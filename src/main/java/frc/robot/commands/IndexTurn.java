// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class IndexTurn extends CommandBase {
  Index index;
  double revolutions;
  double startingRevolutions; 

  public IndexTurn(double revolutions, Index index) {
    this.index = index; 
    this.revolutions = revolutions;
    addRequirements(index);
  }


  @Override
  public void initialize() {
    startingRevolutions = index.getRevolutions();   
    index.on();
  }


  //output = input / gear ratio
  @Override
  public void execute() {
    SmartDashboard.putNumber("index.getRevolutions()", index.getRevolutions()); 
  }

  @Override
  public void end(boolean interrupted) {
    index.off(); 
  }

  @Override
  public boolean isFinished() {
    return  (index.getRevolutions() - startingRevolutions) > revolutions; 
  }
}
