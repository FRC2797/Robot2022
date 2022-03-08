package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.BallData;

public class shooterRevLimelightDistance extends CommandBase {
    Shooter shooter; 
    Limelight limelight; 
    BallData[] blueBallDatas = Constants.blueBallDatas;

    public shooterRevLimelightDistance(Shooter shooter, Limelight limelight) {
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
       return distanceInInches * Constants.shooterSlopeConstant;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }
}
