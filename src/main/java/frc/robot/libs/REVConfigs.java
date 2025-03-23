package frc.robot.libs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.libs.Constants;

public class REVConfigs {

    public final SparkMaxConfig defaultVelocitySparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig defaultPositionSparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig angleSparkMaxConfig = new SparkMaxConfig();

    public REVConfigs() {
    
        /* Swerve Module Drive Motor Configuration */

            /* Motor Invert and Idle Mode */
            
        /* Swerve Module Angle Motor Configuration */

            /* Motor Invert and Idle Mode */
            angleSparkMaxConfig.inverted(Constants.kDrivetrain.ANGLE_INVERT);
            angleSparkMaxConfig.idleMode(Constants.kDrivetrain.ANGLE_IDLE_MODE);

            /* Current Limiting */
            angleSparkMaxConfig.smartCurrentLimit(Constants.kDrivetrain.ANGLE_CURRENT_LIMIT);

            /* Open and Closed Loop Ramping */
            angleSparkMaxConfig.openLoopRampRate(0);
            angleSparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            angleSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(Constants.kDrivetrain.ANGLE_VELOCITY_PERIOD_MS);
            angleSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(Constants.kDrivetrain.ANGLE_POSITION_PERIOD_MS);

            /* Voltage Compensation */
            angleSparkMaxConfig.voltageCompensation(12);

            /* Conversion Factors */
            angleSparkMaxConfig.encoder.positionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
            angleSparkMaxConfig.encoder.velocityConversionFactor(Constants.kDrivetrain.ANGLE_VELOCITY_CONVERSION_FACTOR_DEGREES);

            /* PID */
            angleSparkMaxConfig.closedLoop.p(Constants.kDrivetrain.ANGLE_KP);
            angleSparkMaxConfig.closedLoop.i(Constants.kDrivetrain.ANGLE_KI);
            angleSparkMaxConfig.closedLoop.d(Constants.kDrivetrain.ANGLE_KD);
    }

}