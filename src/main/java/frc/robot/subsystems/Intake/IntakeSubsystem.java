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
import frc.robot.constants.intakeConstants;

public class IntakeSubsystem extends BaseSubsystem {

    private static IntakeSubsystem mInstance;

    private final SparkMax m_pivotMotor;
    private final SparkMax m_intakeMotor;
    private final SparkMax m_algaeShooterMotor;

    private final RelativeEncoder m_pivotEncoder;

    private final DigitalInput m_coralSensor;

    private SparkClosedLoopController m_pivotPIDController;

    private TrapezoidProfile mProfile;
    private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

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
        m_algaeShooterMotor = new SparkMax(intakeConstants.algaeShooterMotorId, intakeConstants.algaeShooterMotorType);
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

        SparkMaxConfig configAlgaeShooter = new SparkMaxConfig();
        configAlgaeShooter.idleMode(intakeConstants.idleMode);
        configAlgaeShooter.smartCurrentLimit(intakeConstants.algaeShooterMotorCurrentLimit);

        m_algaeShooterMotor.configure(configAlgaeShooter, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_coralSensor = new DigitalInput(intakeConstants.proximitySensorId);

        

    }

    @Override
    public void reset() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void writePeriodicOutputs() {
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

    }

}
