package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class intakeConstants{
    public static final int intakeMotorId = 0;
    public static final MotorType intakeMotorType = MotorType.kBrushless;
    public static final int intakeMotorCurrentLimit = 40;

    public static final int pivotMotorId = 0;
    public static final MotorType pivotMotorType = MotorType.kBrushless;
    public static final int pivotMotorCurrentLimit = 10;

    public static final int algaeShooterMotorId = 0;
    public static final MotorType algaeShooterMotorType = MotorType.kBrushless;
    public static final int algaeShooterMotorCurrentLimit = 40;

    public static final IdleMode idleMode = IdleMode.kBrake;

    public static final double pivotMotorMinOutput = -0.2;
    public static final double pivotMotorMaxOutput = 0.2;


    public static final int profileMaxVelocity = 65;
    public static final int profileMaxAccel = 100;


    public static final int proximitySensorId = 0;

    public static final double p = 0;
    public static final double i = 0;
    public static final double d = 0;
    public static final double iZone = 0;


    //Speeds
    public static final double CoralIntakeSpeed = -0.6;
        
    public static final double AlgeaIntakeSpeed = -0.7;
    
    public static final double AlgeaShootSpeed = 0.8;

}