package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class elevatorConstant {
    public static final int leftMotorId = 2;
    public static final int rightMotorId = 4;

    public static final int limitSwitcherId = 2;

    public static final MotorType leftMotorType = MotorType.kBrushless;
    public static final MotorType rightMotorType = MotorType.kBrushless;

    public static final IdleMode idleMode = IdleMode.kBrake;

    //PID values
    public static final double p = 0.085;
    public static final double i = 0;
    public static final double d = 0.05;
    public static final double iZone = 5;

    //Trapezoid Profile Constants
    public static final double maxVelocity = 65;
    public static final double maxAcceleration = 200;

    public static final boolean inverted = false;

    //Levels
    public static final double kElevatorL1 = 14;
    public static final double kElevatorL2 = 30;
    public static final double kElevatorL3 = 54;
    public static final double kElevatorL4 = 0;
        
    public static final double kElevatorAlgeaLeveL1 = 0;
    public static final double kElevatorAlgeaLeveL2 =   0;

    public static final double kToleranceElevator = 1;
}
