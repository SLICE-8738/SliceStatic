package frc.robot.subsystems.Swerve;
import frc.robot.Robot;
import frc.robot.libs.CTREConfigs;
import frc.robot.libs.Constants;
import frc.robot.libs.Conversions;
import frc.robot.libs.OnboardModuleState;
import frc.robot.libs.SwerveModuleConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private SwerveModuleState targetState = new SwerveModuleState();

    private SparkMax angleMotor;
    private TalonFX driveMotor;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private SparkMaxConfig config = new SparkMaxConfig();;

    private final CTREConfigs ctreConfigs = new CTREConfigs();
    private final SparkClosedLoopController angleController;
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(frc.robot.libs.Constants.kDrivetrain.DRIVE_KS, frc.robot.libs.Constants.kDrivetrain.DRIVE_KV, frc.robot.libs.Constants.kDrivetrain.DRIVE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private double simDistance = 0;
    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();
        //angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID , MotorType.kBrushless);
        angleMotor.configure(Constants.REV_CONFIGS.angleSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        angleMotor.setCANTimeout(200);
        configAngleMotor();
        //configAngleMotor();*

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        targetState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        /*desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);*/
    }

    public void setPercentOutput(double drivePercentOutput, double anglePercentOutput) {
        driveDutyCycle.Output = drivePercentOutput;
        driveMotor.setControl(driveDutyCycle);
        angleMotor.set(anglePercentOutput);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / frc.robot.libs.Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToTalon(desiredState.speedMetersPerSecond, frc.robot.libs.Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private Rotation2d waitForCANcoder(){
        /* wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = waitForCANcoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    } //setPosition
    
    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);
    }

    private void configAngleMotor(){
        config.encoder.positionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
        config.closedLoop.pidf(Constants.kDrivetrain.ANGLE_KP, Constants.kDrivetrain.ANGLE_KI, Constants.kDrivetrain.ANGLE_KD, Constants.kDrivetrain.ANGLE_KFF);
        angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0);
        driveMotor.getVelocity().setUpdateFrequency(Constants.kDrivetrain.DRIVE_VELOCITY_FRAME_RATE_HZ);
        driveMotor.getPosition().setUpdateFrequency(Constants.kDrivetrain.DRIVE_POSITION_FRAME_RATE_HZ);
    }

    public void setDriveIdleMode(boolean setBrakeMode) {

        driveMotor.setNeutralMode(setBrakeMode? NeutralModeValue.Brake : NeutralModeValue.Coast);

    }

    public void setAngleIdleMode(boolean setBrakeMode) {

        config.idleMode(setBrakeMode? IdleMode.kBrake : IdleMode.kCoast);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.talonToMPS(driveMotor.getVelocity().getValueAsDouble(), frc.robot.libs.Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.talonToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public void setSimulationPosition() {
        simDistance += driveMotor.getVelocity().getValueAsDouble() * 0.02;
        driveMotor.setPosition(simDistance);
        integratedAngleEncoder.setPosition(lastAngle.getDegrees());
    }
}