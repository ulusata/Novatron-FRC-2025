package frc.lib.abstracts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.enums.TelemetryVerbosityLevel;

public abstract class BaseSubsystem extends SubsystemBase {

    private TelemetryVerbosityLevel verbosityLevel = TelemetryVerbosityLevel.NONE;

    /**
     * Sets the telemetry verbosity level for this subsystem.
     */
    public void setTelemetryVerbosity(TelemetryVerbosityLevel level) {
        verbosityLevel = level;
    }

    /**
     * Writes the relevant subsystem information to the log
     */
    public void writeToLog() {
    }

    /**
     * Reset sensors, PID Controllers, and any private instance variables
     */
    public abstract void reset();

    /**
     * Contains the algorithms and logic to update the values in mPeriodicIO
     * <p>
     * Do not set the outputs of the system here!!
     * <p>
     * This function is called each periodic cycle before the
     * {@link #writePeriodicOutputs()} function
     */
    public abstract void periodic();

    /**
        Simulates the subsystem in simulation mode
     */
    public abstract void simulationPeriodic();

    /**
     * <p>
     * Using the current values in mPeriodicIO, sets the outputs of this subsystem.
     * <p>
     * Examples:
     *
     * <pre>
     * mMotor.set(mPeriodicIO.speed);
     * mSolenoid.set(mPeriodicIO.open);
     * mLED.setRGB(...);
     * etc...
     * </pre>
     * <p>
     * The value of mPeriodicIO variables should not be changed in this function.
     * <p>
     * This function is called each periodic cycle after the {@link #periodic()}
     * function
     */
    public abstract void writePeriodicOutputs();

    /**
     * Stops the subsystem, putting it in a state that is safe for people to touch
     * the robot.
     * <p>
     * Called once when the robot is entering the disabled state
     * {@link #disabledInit()}
     */
    public abstract void stop();

    /**
     * Outputs the low verbosity telemetry. Override in subclasses for specific
     * details.
     */
    protected abstract void outputLowTelemetry();

    /**
     * Outputs the high verbosity telemetry. Override in subclasses for specific
     * details.
     */
    protected abstract void outputHighTelemetry();

    /**
     * Puts the relevant subsystem information on the SmartDashboard based on the
     * verbosity level.
     */
    public void outputTelemetry() {
        switch (verbosityLevel) {
            case NONE:
                // No telemetry output
                break;
            case LOW:
                outputLowTelemetry();
                break;
            case HIGH:
                // Include both LOW and HIGH telemetry for high verbosity
                outputLowTelemetry();
                outputHighTelemetry();
                break;
        }
    }

}