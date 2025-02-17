package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class elevatorConstant {
    public static int leftMotorId = 2;
    public static int rightMotorId = 4;

    public static int limitSwitcherId = 2;

    public static MotorType leftMotorType = MotorType.kBrushless;
    public static MotorType rightMotorType = MotorType.kBrushless;

    public static IdleMode idleMode = IdleMode.kBrake;

    //PID values
    public static double p = 0.085;
    public static double i = 0;
    public static double d = 0;
    public static double iZone = 5;

    //Trapezoid Profile Constants
    public static double maxVelocity = 65;
    public static double maxAcceleration = 200;

    public static boolean inverted = false;

}
