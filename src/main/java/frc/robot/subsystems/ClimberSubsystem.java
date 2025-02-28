// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.abstracts.BaseSubsystem;
import frc.lib.enums.TelemetryVerbosityLevel;
import frc.robot.constants.climberConstant;

public class ClimberSubsystem extends BaseSubsystem {

  private SparkMax ClimberMotor;

  private RelativeEncoder ClimberMotorEncoder;

  private ClimberIO climberIO;

  public ClimberSubsystem() {
    setTelemetryVerbosity(TelemetryVerbosityLevel.HIGH);

    ClimberMotor = new SparkMax(climberConstant.climberID, MotorType.kBrushless);

    ClimberMotorEncoder = ClimberMotor.getEncoder();

    SparkMaxConfig configClimberMotor = new SparkMaxConfig();

    configClimberMotor.smartCurrentLimit(40);
    configClimberMotor.idleMode(IdleMode.kBrake);

    ClimberMotor.configure(configClimberMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public static class ClimberIO{
    double speed = 0;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    ClimberMotor.set(climberIO.speed);
  }

  @Override
  public void stop() {

  }

  @Override
  public void reset() {

  }

  @Override
  public void simulationPeriodic() {

  }
  
  @Override
  public void outputLowTelemetry() {

  }

  @Override
  public void outputHighTelemetry() {

  }


  public void setSpeed(double speed){
    climberIO.speed = MathUtil.clamp(speed, -1, 1);
  }

  public void stopMotors(){
    climberIO.speed = 0;
  }

}
