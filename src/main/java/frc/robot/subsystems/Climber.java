package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Motors.Climber.*; 


public class Climber extends SubsystemBase {
    final private CANSparkMax frontLeft = new CANSparkMax(FrontLeft, MotorType.kBrushless);
    final private CANSparkMax rearLeft = new CANSparkMax(RearLeft, MotorType.kBrushless);
    final private CANSparkMax frontRight = new CANSparkMax(FrontRight, MotorType.kBrushless);
    final private CANSparkMax rearRight = new CANSparkMax(RearRight, MotorType.kBrushless);

    final private MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, rearLeft);
    final private MotorControllerGroup rightGroup = new MotorControllerGroup(frontRight, rearRight);

    public Climber() {

    }

    

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
