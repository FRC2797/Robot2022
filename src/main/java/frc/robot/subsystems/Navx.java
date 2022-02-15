// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
    private AHRS ahrs = new AHRS();

    public Navx() {
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Angle", ahrs.getAngle());
        SmartDashboard.putNumber("Displacement X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement Y", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Displacement Z", ahrs.getDisplacementZ());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
        SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Distance in meters", getDistance());
        SmartDashboard.putBoolean("Is Calibrating", ahrs.isCalibrating());
    }

    @Override
    public void simulationPeriodic() {
    }

    public void getAngle() {
        ahrs.getAngle();
    }

    public double getDistance() {
        // a^2 + b^2 = c^2
        // sqrt((a^2) + (b^2)) = c

        //returns it in meters
        return Math.sqrt((ahrs.getDisplacementX() * ahrs.getDisplacementX())
                + (ahrs.getDisplacementY() * ahrs.getDisplacementY()));
    }

    //FIXME: It might not be roll, it might be pitch or yaw
    public double getRotation() {
        return ahrs.getAngle();
    } 

    public void reset() {
        ahrs.reset();
        ahrs.resetDisplacement();
        ahrs.calibrate();
    }
}
