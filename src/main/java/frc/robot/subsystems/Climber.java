package frc.robot.subsystems;

import static frc.robot.Constants.Motors.Climber.FrontLeft;
import static frc.robot.Constants.Motors.Climber.FrontRight;
import static frc.robot.Constants.Motors.Climber.RearLeft;
import static frc.robot.Constants.Motors.Climber.RearRight;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    final private CANSparkMax frontLeft = new CANSparkMax(FrontLeft, MotorType.kBrushless);
    final private CANSparkMax rearLeft = new CANSparkMax(RearLeft, MotorType.kBrushless);
    final private CANSparkMax frontRight = new CANSparkMax(FrontRight, MotorType.kBrushless);
    final private CANSparkMax rearRight = new CANSparkMax(RearRight, MotorType.kBrushless);

    final private MotorControllerGroup frontGroup = new MotorControllerGroup(frontLeft, frontRight);
    final private MotorControllerGroup rearGroup = new MotorControllerGroup(rearLeft, rearRight);

    public Climber() {
        brake();
        frontLeft.setInverted(true);
    }


    public void setFrontLeftUp() {
        frontLeft.set(Constants.climberSpeed);
    }

    public void setFrontLeftDown() {
        frontLeft.set(-Constants.climberSpeed);
    }

    public void frontLeftOff() {
        frontLeft.set(0);
    }

    public void setFrontRightUp() {
        frontRight.set(Constants.climberSpeed);
    }

    public void setFrontRightDown() {
        frontRight.set(-Constants.climberSpeed);
    }

    public void frontRightOff() {
        frontRight.set(0);
    }

    public void setRearLeftUp() {
        rearLeft.set(Constants.climberSpeed);
    }

    public void setRearLeftDown() {
        rearLeft.set(-Constants.climberSpeed);
    }

    public void rearLeftOff() {
        rearLeft.set(0);
    }

    public void setRearRightUp() {
        rearRight.set(Constants.climberSpeed);
    }

    public void setRearRightDown() {
        rearRight.set(-Constants.climberSpeed);
    }

    public void rearRightOff() {
        rearRight.set(0);
    }




    public void setFrontUp() {
        frontGroup.set(Constants.climberSpeed);
    }

    public void setFrontDown() {
        frontGroup.set(-Constants.climberSpeed);
    }

    public void frontOff() {
        frontGroup.set(0);
    }

    public void setRearUp() {
        rearGroup.set(Constants.climberSpeed);
    }

    public void setRearDown() {
        rearGroup.set(-Constants.climberSpeed);
    }

    public void rearOff() {
        rearGroup.set(0);
    }

    public void brake() {
        frontLeft.setIdleMode(IdleMode.kBrake);
        rearLeft.setIdleMode(IdleMode.kBrake);
        frontRight.setIdleMode(IdleMode.kBrake);
        rearRight.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
