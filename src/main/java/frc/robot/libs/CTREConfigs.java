package frc.robot.libs;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.kDrivetrain.DRIVE_INVERT;
        driveMotorOutput.NeutralMode = Constants.kDrivetrain.DRIVE_IDLE_MODE;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.kDrivetrain.DRIVE_CURRENT_LIMIT;
        //swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.kDrivetrain.DRIVE_CURRENT_THRESHOLD;
        //swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.kDrivetrain.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = Constants.kDrivetrain.DRIVE_KP;
        driveSlot0.kI = Constants.kDrivetrain.DRIVE_KI;
        driveSlot0.kD = Constants.kDrivetrain.DRIVE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.kDrivetrain.CANCODER_INVERT;
       // swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    }
}