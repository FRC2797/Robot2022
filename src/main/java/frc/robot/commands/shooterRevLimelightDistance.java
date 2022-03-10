package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.BallData;

public class ShooterRevLimelightDistance extends CommandBase {
    Shooter shooter; 
    Limelight limelight; 
    BallData[] blueBallDatas = Constants.blueBallDatas;
    private static ShuffleboardTab driverTab;
    private static NetworkTableEntry shooterSlope;

    public ShooterRevLimelightDistance(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight; 
        withName("shooterRevLimelightDistance");
        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(percFromDist(limelight.getDistance()));
    }

    //public void end;

    public double percFromDist(double distanceInInches) {
       return ( (distanceInInches / shooterSlope.getDouble(0))/100  );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }

    public static void setUpShuffleboardShooterSlopeValue() {
        driverTab = Shuffleboard.getTab("driver");
        shooterSlope = driverTab.add("shooterSlope", Constants.shooterSlopeConstant).getEntry();
    } 
}
