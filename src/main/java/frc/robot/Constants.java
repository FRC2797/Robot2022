// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Motors {
        public static final int kRearRight = 1;
        public static final int kFrontRight = 2;
        public static final int kFrontLeft = 8;
        public static final int kRearLeft = 4;
        public static final int kIntake = 5;
        public static final int kIndex = 6;
        public static final int kShooter1 = 7;
        public static final int kShooter2 = 9;

    }

    public static final double kdeadband = 0.1;
    public static final double kfilterLimitForwards = 0.5;
    public static final double kfilterLimitSideways = 0;
    public static final double kfilterLimitRotation = 0;

    // Limelight constants
    // All in inches    
    public static final double hubHeight = 96;
    public static final double limelightHeight = 39 + (3/4);
    public static final double heightDifference = hubHeight - limelightHeight;
    public static final double mountingAngle = 8.25;

    //Control constants
    public static final double aimingkP = 0.1;
    public static final double aimingkI = 0;
    public static final double aimingkD = 0;
    public static final double minimum = 0; 


    public static final double indexGearRatio = 0.5; 
    public static final double indexWaitTime = 0.5;

    //Auto Constants
    public static final double forwardTime = 0;
    public static final double rotateTime = 0;
}
