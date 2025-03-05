package frc.robot.subsystems.Elavator;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.abstracts.BaseSubsystem;
import frc.lib.enums.TelemetryVerbosityLevel;
import frc.robot.constants.elevatorConstant;


public class ElevatorSubsystem extends BaseSubsystem{

    private static ElevatorSubsystem mInstance;

    ElevatorIOInputs elevatorIO;

    private SparkMax m_leftNeoMotor;
    private SparkMax m_rightNeoMotor;

    private RelativeEncoder m_leftNeoMotorEncoder;

    private SparkClosedLoopController mLeftPIDController;
    private DigitalInput limitSwitch;

    private TrapezoidProfile mProfile;
    private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    public ElevatorSubsystem(){
        setTelemetryVerbosity(TelemetryVerbosityLevel.HIGH);

        m_leftNeoMotor = new SparkMax(elevatorConstant.leftMotorId, MotorType.kBrushless);
        m_rightNeoMotor = new SparkMax(elevatorConstant.rightMotorId, MotorType.kBrushless);

        m_leftNeoMotorEncoder = m_leftNeoMotor.getEncoder();

        mLeftPIDController = m_leftNeoMotor.getClosedLoopController();

        SparkMaxConfig configLeftNeoMotorEncoder = new SparkMaxConfig();

        configLeftNeoMotorEncoder.closedLoop.pid(
                0.085,
                0,
                0)
                .iZone(5)
                .minOutput(-0.5)
                .maxOutput(0.5 );

        configLeftNeoMotorEncoder.smartCurrentLimit(40);
        configLeftNeoMotorEncoder.idleMode(IdleMode.kBrake);

        m_leftNeoMotor.configure(configLeftNeoMotorEncoder, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_rightNeoMotor.configure(configLeftNeoMotorEncoder.follow(elevatorConstant.leftMotorId, elevatorConstant.inverted),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorIO = new ElevatorIOInputs();
        mProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(65, 200));

        limitSwitch = new DigitalInput(elevatorConstant.limitSwitcherId);

    }

    private static class ElevatorIOInputs{
        boolean elevator_pos_control = false;
        double elevatorVolt = 0;
        double elevator_target = 0;
    }

    @Override
    public void periodic(){
        if (!limitSwitch.get()) {
            m_leftNeoMotorEncoder.setPosition(0);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - prevUpdateTime;
        prevUpdateTime = curTime;
        if (elevatorIO.elevator_pos_control) {
            mGoalState.position = elevatorIO.elevator_target;

            prevUpdateTime = curTime;
            mCurState = mProfile.calculate(dt, mCurState, mGoalState);

            mLeftPIDController.setReference(
                    mCurState.position,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    0.75,
                    ArbFFUnits.kVoltage);
        } else {
            m_leftNeoMotor.setVoltage(elevatorIO.elevatorVolt);
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void simulationPeriodic() {
        m_leftNeoMotorEncoder.setPosition(SmartDashboard.getNumber("angle ELV", 0));

    }

    @Override
    protected void outputLowTelemetry() {

    }

    @Override
    protected void outputHighTelemetry() {
        SmartDashboard.putNumber("elevator Position", m_leftNeoMotorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Target Position", elevatorIO.elevator_target);
        SmartDashboard.putNumber("Elevator Voltage", m_leftNeoMotor.getBusVoltage());
        SmartDashboard.putBoolean("elevator Limit Switch", limitSwitch.get());
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.elevator_pos_control = false;
        elevatorIO.elevatorVolt = voltage;
    }

    public void goToLevel(double level){
        elevatorIO.elevator_pos_control = true;
        elevatorIO.elevator_target = level;
    }

    public double getPosition(){
        return m_leftNeoMotorEncoder.getPosition();
    }

    public boolean isAtLevel(double level){
        return Math.abs(m_leftNeoMotorEncoder.getPosition() - (level)) < elevatorConstant.kToleranceElevator;
    }
}
