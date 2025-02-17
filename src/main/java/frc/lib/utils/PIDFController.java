/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Custom PID Controller
 */
public class PIDFController extends PIDController {
  
  private double kF;

  public PIDFController(double kP, double kI, double kD, double kF, boolean enabled) {
    super(kP, kI, kD);
    this.kF = kF;
  }

  public PIDFController(double kP, double kI, double kD, double kF) {
      this(kP, kI, kD, kF, true);
  }

  public double getFeedForward() {
      return kF;
  }

  public void setFeedForward(double kF) {
      this.kF = kF;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  @Override
  public double calculate(double measurement) {
      double otpt = super.calculate(measurement);
      if(otpt > 0.0)
        otpt += this.kF;
      else if(otpt < 0.0)
        otpt -= this.kF;
      return otpt;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint    The new setpoint of the controller.
   */
  @Override
  public double calculate(double measurement, double setpoint) {
      // Set setpoint to provided value
      super.setSetpoint(setpoint);
      return super.calculate(measurement);
  }

  public void debug() {
      SmartDashboard.putData(this);

  }
}
