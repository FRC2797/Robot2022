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
        public final class Drivetrain {
            public static final int RearRight = 1;
            public static final int FrontRight = 2;
            public static final int FrontLeft = 8;
            public static final int RearLeft = 4;
        }

        public final class Climber {
            //FIXME: Needs actual constants
            public static final int RearRight = 16;
            public static final int FrontRight = 17;
            public static final int FrontLeft = 18;
            public static final int RearLeft = 19;
        }

        public static final int Intake = 5;
        public static final int Index = 6;
        public static final int leftShooter = 7;
        public static final int rightShooter = 9;
        public static final int LeftClimber = 10;
        public static final int RightClimber = 11;

    }

    public static final double deadband = 0;
    public static final double filterLimitForwards = 0.5;
    public static final double filterLimitSideways = 0;
    public static final double filterLimitRotation = 0;

    // Xbox Controller Constants
    public static final double triggerDeadzone = 0.05;
    public static final double drivingDeadzone = 0.2;

    // Auto Constants
    // TODO: Need to get right constant, look at field, might need multiple so that
    // we can start in multiple spots
    public static final double autoDriveDistance = 0;

    // Limelight constants
    // All in inches
    public static final double hubHeight = 90;
    public static final double limelightHeight = 39 + (3 / 4);
    public static final double heightDifference = hubHeight - limelightHeight;
    public static final double mountingAngle = 8.25;

    // Control Engineering constants
    public static final double aimingkP = 0.1;
    public static final double aimingkI = 0;
    public static final double aimingkD = 0;
    public static final double minimum = 0;

    public static final double driveDistanceTolerance = 0.1;
    public static final double driveDistancekP = 0.01;

    // Index
    public static final double indexWaitTime = 0.5;
    public static final double indexPower = -0.6;
    public static final double indexFromIntakeRevolutions = 2.25;
    public static final double indexIntoShooterRevolutions = 0.33;

    // Shooter
    public static final double shooterSpinUpTime = 0.1;
    public static final double shooterGearRatio = 1;


    // Auto Constants
    public static final double forwardTime = 0;
    public static final double rotateTime = 0;

    public final class encoderConstantsDrivetrain {
        // Motor used: NEO Brushless, REV-21-1650
        // Wheel diameter in inches
        public static final double wheelDiameter = 6;
        public static final double wheelCircumference = 6 * Math.PI;
        public static final double gearRatio = 7.31;

        // this is correct, The relative encoder getPosition() returns the amount of
        // input rotations directly, no need to change it whatsoever
        public static final double outputRotationInAInputRotation = 1 / gearRatio;
    }

    public final class encoderConstantsIndex {
        // Motor used: NEO Brushless, REV-21-1650
        // Wheel diameter in inches
        public static final double wheelDiameter = 4;
        public static final double gearRatio = 12.75;

        // this is correct, The relative encoder getPosition() returns the amount of
        // input rotations directly, no need to change it whatsoever
        public static final double outputRotationInAInputRotation = 1 / gearRatio;
    }
}
