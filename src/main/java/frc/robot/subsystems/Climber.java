package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(Constants.Motors.LeftClimber, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.Motors.RightClimber, MotorType.kBrushless);

    public Climber() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
