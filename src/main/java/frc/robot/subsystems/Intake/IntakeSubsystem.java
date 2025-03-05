package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.abstracts.BaseSubsystem;
import frc.lib.enums.TelemetryVerbosityLevel;
import frc.robot.constants.elevatorConstant;
import frc.robot.constants.intakeConstants;
import frc.robot.RobotState.IntakeState;

public class IntakeSubsystem extends BaseSubsystem {

    private static IntakeSubsystem mInstance;

    private final SparkMax m_pivotMotor;
    private final SparkMax m_intakeMotor;

    private final RelativeEncoder m_pivotEncoder;

    private final DigitalInput m_coralSensor;
    private final DigitalInput m_limitSwitch;

    private SparkClosedLoopController m_pivotPIDController;

    private TrapezoidProfile mProfile;
    private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

    private IntakeIOInputs intakeIO;

    public static IntakeSubsystem getInstance()
    {
        if(mInstance == null){
            return new IntakeSubsystem();
        }
        return mInstance;
    }

    public IntakeSubsystem(){
        setTelemetryVerbosity(TelemetryVerbosityLevel.HIGH);

        m_intakeMotor = new SparkMax(intakeConstants.intakeMotorId, intakeConstants.intakeMotorType);
        m_pivotMotor = new SparkMax(intakeConstants.pivotMotorId, intakeConstants.pivotMotorType);
        m_pivotEncoder = m_pivotMotor.getEncoder();
        m_pivotPIDController = m_pivotMotor.getClosedLoopController();

        SparkMaxConfig configIntakeMotor = new SparkMaxConfig();
        configIntakeMotor.idleMode(intakeConstants.idleMode);
        configIntakeMotor.smartCurrentLimit(intakeConstants.intakeMotorCurrentLimit);

        m_intakeMotor.configure(configIntakeMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig configPivotMotor = new SparkMaxConfig();

        configPivotMotor.idleMode(intakeConstants.idleMode);
        configPivotMotor.smartCurrentLimit(intakeConstants.pivotMotorCurrentLimit);

        configPivotMotor.closedLoop.pid(
                intakeConstants.p,
                intakeConstants.i,
                intakeConstants.d)
                .iZone(intakeConstants.iZone)
                .minOutput(intakeConstants.pivotMotorMinOutput)
                .maxOutput(intakeConstants.pivotMotorMaxOutput);

        m_pivotMotor.configure(configPivotMotor, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_coralSensor = new DigitalInput(intakeConstants.proximitySensorId);

        m_limitSwitch = new DigitalInput(intakeConstants.limitSwitcherId);

        intakeIO = new IntakeIOInputs();

        mProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(elevatorConstant.maxVelocity, elevatorConstant.maxAcceleration));


    }

    private static class IntakeIOInputs {
        double intake_speed = 0.0;
        double intake_pivot_angle = 0.0;
        double intake_pivot_voltage = 0.0;
        boolean is_intake_pos_control = true;
    }

    @Override
    public void reset() {

    }

    @Override
    public void periodic(){
         if (!m_limitSwitch.get()) {
            m_pivotEncoder.setPosition(0);
        }

        if (IntakeState.isEmpty == false && !getCoralSensor()) {
            m_intakeMotor.set(0.3);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - prevUpdateTime;
        prevUpdateTime = curTime;
        if (intakeIO.is_intake_pos_control) {

            mGoalState.position = intakeIO.intake_pivot_angle;
            prevUpdateTime = curTime;
            mCurState = mProfile.calculate(dt, mCurState, mGoalState);

            m_pivotPIDController.setReference(
                    mCurState.position,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
        } else {
            m_pivotMotor.setVoltage(intakeIO.intake_pivot_voltage);
        }
        m_intakeMotor.set(intakeIO.intake_speed);
    }

    @Override
    public void stop() {

    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    protected void outputLowTelemetry() {

    }

    @Override
    protected void outputHighTelemetry() {
        SmartDashboard.putBoolean("coral", getCoralSensor());
        SmartDashboard.putNumber("intake angle", getPivotAngle());
       SmartDashboard.putBoolean("intake switch", m_limitSwitch.get()); 
    }

    public void setIntakeSpeed(double speed) {
        intakeIO.intake_speed = speed;
    }

    public void setIntakePivotVoltage(double voltage) {
        intakeIO.is_intake_pos_control = false;
         intakeIO.intake_pivot_voltage = voltage;
     }

    public void stopIntake() {
        intakeIO.intake_speed = 0.0;
    }

    public double getPivotAngle() {
        return m_pivotEncoder.getPosition();
    }

    public boolean getIntakeMotorState() {
        return Math.abs(intakeIO.intake_speed) > 0.0;
    }

    public boolean getCoralSensor() {
        return m_coralSensor.get();
    }

    public void setPositionUp(){
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle += 0.01;
    }

    public void setPositionDown(){
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle -= 0.01;
    }

    public void setPivotBaseLevel() {
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle = intakeConstants.kBaseAngle;
    }

    public void setDropLevel() {
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle = intakeConstants.kCoralAngle;
    }

    public void setAlgaeLevel()
    {
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle = intakeConstants.algaeLevel;
    }
    public void setL4Level() {
        intakeIO.is_intake_pos_control = true;
        intakeIO.intake_pivot_angle = intakeConstants.kL4Angle;
    }

    public boolean isAtDropAngle() {
        return Math.abs(m_pivotEncoder.getPosition() - 0) < intakeConstants.kTolerancePivot;
    }

    public boolean isAtBaseAngle() {
        return Math.abs(m_pivotEncoder.getPosition() - intakeConstants.kBaseAngle) < intakeConstants.kTolerancePivot;
    }

    public boolean isAtL4Level() {
        return Math.abs(
                m_pivotEncoder.getPosition() - intakeConstants.kL4Angle) < intakeConstants.kTolerancePivot;
    }

    public boolean ifAngleLessThanBaseAngle() {
        return m_pivotEncoder.getPosition() < intakeConstants.kBaseAngle + intakeConstants.kTolerancePivot;
    }

}
