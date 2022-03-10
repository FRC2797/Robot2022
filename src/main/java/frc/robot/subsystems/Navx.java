// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
    private AHRS ahrs = new AHRS();
    
    public Navx() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public void getAngle() {
        ahrs.getAngle();
    }

    //doesn't work
    public double getDistance() {
        // a^2 + b^2 = c^2
        // sqrt((a^2) + (b^2)) = c      a

        //returns it in meters
        return Math.sqrt((ahrs.getDisplacementX() * ahrs.getDisplacementX())
                + (ahrs.getDisplacementY() * ahrs.getDisplacementY()));
    }

    
    public double getRotation() {
        return ahrs.getAngle();
    } 

    public void reset() {
        ahrs.reset();
        ahrs.resetDisplacement();
        ahrs.calibrate();
    }

    public boolean isConnected() {
        return ahrs.isConnected();
    }
}
