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
        shooter.setSpeed(percFromDistBlue(limelight.getDistance()));
    }

    //public void end;

    public double percFromDistBlue(double distance) {
        blueBallDatas = Constants.blueBallDatas;
        BallData lowerBall;
        BallData higherBall;
        int closestIndex = 0;
        BallData closestBall = null; 

        for (int i = 0; i < blueBallDatas.length; i--) {
            closestBall = blueBallDatas[0];
            BallData currentBall = blueBallDatas[i];  
            //find the closest ball
            if (Math.abs(closestBall.distance - distance) > Math.abs(currentBall.distance - distance)) {
                closestBall  = currentBall;
                closestIndex = i;
            }
        }

        if () {

        }

        // BallData lowerBall = ;
        // BallData higherBall = ;

        return closestBall.percentage; 
    }  

    public double percFromDistRed(double distance) {
        return 0;
    }
}
