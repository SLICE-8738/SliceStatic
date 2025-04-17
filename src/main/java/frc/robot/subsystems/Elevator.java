// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  public SparkMax leader = new SparkMax(11, MotorType.kBrushless);
  public SparkMax follower = new SparkMax(12, MotorType.kBrushless);
  SparkMaxConfig leadConfig = new SparkMaxConfig();
  SparkMaxConfig followConfig = new SparkMaxConfig(); 
  
  //private RelativeEncoder encoder = leader.getEncoder();
  //EncoderConfig encoderConfig = new EncoderConfig();

  private SparkClosedLoopController controller = leader.getClosedLoopController();
  
  public double[] heights = {24.000, 31.875, 47.625, 48, 16}; //height in inches
  //                         L1      L2      L3      L4  grab
  public double currentPosition = 0;

  //speed conversion = 5676/5

  public Elevator(){
    configureMotors();
  }

  private void configureMotors(){
    leadConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    leadConfig.encoder
       .positionConversionFactor(1.65)
       .velocityConversionFactor(1.65);
       //.velocityConversionFactor(1000);//(5*0.0508)/42);
    leadConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(.02, 0, 0.002);
    followConfig.apply(leadConfig);
    followConfig.follow(11, false);

    leader.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //encoderConfig.positionConversionFactor((5*0.0508)/42);
  }

  public void setPosition(double p){
    currentPosition = p;
    // System.out.println(currentPosition);
    // System.out.println(encoder.getPosition());
    controller.setReference(p, SparkMax.ControlType.kPosition);    
    
    //leader.set(.1); //for testing
  }

  public void stop(){
    //leader.set(0); //for testing
  }

  //Commands for use when constructing Autos
  public Command setPosition0Auto(){
    return runOnce(() -> setPosition(heights[0]));
  }

  public Command setPosition1Auto(){
    return runOnce(() -> setPosition(heights[1]));
  }

  public Command setPosition2Auto(){
    return runOnce(() -> setPosition(heights[2]));
  }

  public Command setPosition3Auto(){
    return runOnce(() -> setPosition(heights[3]));
  }



/************* ROBOCATS 1699 ************/

// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// private RelativeEncoder encoder = leader.getEncoder();
//   private SparkClosedLoopController feedbackController = leader.getClosedLoopController();
//   public ElevatorPosition currentTargetPosition = ElevatorPosition.STORED;
//   public final double kTolerance = 1.0;

//   // COMMAND FACTORIES TO REACH ENUM HEIGHT
//     public Command setRaw(double percent) {
//         return runOnce(() -> {
//             leader.set(percent);
//         });
//     }

//     /** Sets the target height of the elevator. 
//      * @param ElevatorPosition
//      * The taregt position: including state and height.
//     */
//     public Command setPosition(ElevatorPosition position) {
//         return runOnce(() -> {
//             // CHANGES CURRENT TARGET TO POS
//             currentTargetPosition = position;
//             // SETS FEEDBACKCONTROLLER TO POS
//             feedbackController.setReference(position.rotations, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
//         });
//     }

//     /** Sets the target height of the elevator using trapezoidal profiling. 
//      * @param ElevatorPosition
//      * The taregt position: including state and height.
//     */
//     public Command setPositionSmartMotion(ElevatorPosition position) {
//         return runOnce(() -> {
//             // CHANGES CURRENT TARGET TO POS
//             currentTargetPosition = position;
//             // SETS FEEDBACKCONTROLLER TO POS
//             feedbackController.setReference(position.rotations, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
//         });
//     }

//     /**Waits until elevator reaches position within Tolerance.
//      * @param ElevatorPosition
//      * Enum for elevator height options. 
//      */
//     public Command waitUntilAtSetpoint() {
//         return new WaitUntilCommand(() -> {
//             // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
//             return isAtSetpoint();
//         });
//     }
    
//     public boolean isAtSetpoint() {
//         return (getElevatorError() < kTolerance);
//     }

//     private double getElevatorError() {
//         return Math.abs(Math.abs(encoder.getPosition()) - Math.abs(currentTargetPosition.rotations));
//     }

//     /**Resets encoder to 0*/
//     public Command resetEncoder() {
//         return runOnce(() -> {
//                 encoder.setPosition(0);
//             });
//     }
//     /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
//     public Command stopMotorCommand() {
//         return runOnce(() -> {
//           leader.set(0);
//         });
//     }

//     /** Stops the motor manually, ignoring all commands. */
//     public void stopMotorManual() {
//       leader.set(0);
//     }

//     public Command printPosition() {
//         return runOnce(() -> System.out.println(encoder.getPosition()));
//     }

//     public void setIdleMode(IdleMode idleMode) {
//         leadConfig.idleMode(idleMode);
//         leader.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//         followConfig.idleMode(idleMode);
//         follower.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     }
    
//     /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
//     public enum ElevatorPosition {
//         // ENUMS FOR POSITIONS
//         STORED(0), PRIME(-1), COBRA_STANCE(-1),
//         PID_TESTING(20),

//         ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(-1), ALGAE_DESCORE_L_THREE(-1),
      
//         GROUND_INTAKE(7), CORAL_STATION_INTAKE(0),

//         L_ONE(0), L_TWO(2.5), L_THREE(11), L_FOUR(47);

//         private double rotations;
//         /**Constrcutor for height for ElevatorPositions (Enum for Elevator poses)
//         * @param rotations
//         * verticle movement in centimeters
//         */
//         ElevatorPosition(double rotations) {
//             this.rotations = rotations;
//         }

//         public double getRotations() {
//             return this.rotations;
//         }
//     }



/************* BREAD 5940 ************/  
/** Not fully updated */

  // import frc.robot.LoggedTunableNumber;

  // LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.5);
  // LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 50.0);
  // LoggedTunableNumber kForwardsKa = new LoggedTunableNumber("Elevator/kForwardskA", 0.01);
  // LoggedTunableNumber kBackwardsKa = new LoggedTunableNumber("Elevator/kBackwardskA", 0.005);
  // LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Elevator/kMaxVelocity", 3.435000);
  // LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Elevator/kMotionCruiseVelocity", 3.0); 
  // LoggedTunableNumber kMaxAccel = new LoggedTunableNumber("Elevator/kMaxAccel", 12.0); 

  // double lastVelocityTarget, mLastCommandedPosition, posMeters, velMetersPerSecond, velTarget, posTarget, appliedVoltage;
  // double[] currentAmps, tempCelcius;

  // public static final double SECOND_STAGE_HEIGHT = 0.5872286463821073;
  // public static final double ELEVATOR_GEARING = (9.0/44.0);
  // public static final double ELEVATOR_PULLEY_PITCH_DIAMETER = 0.06096;
  // public static final double ELEVATOR_BELOW_STAGE1_KG = -0.004;
  // public static final double ELEVATOR_ABOVE_STAGE1_KG = 0.028;


  // public void updateInputs() {
  //     posMeters = getHeight();
  //     velMetersPerSecond = integratedSensorUnitsToMetersPerSecond(leader.getSelectedSensorVelocity());
  //     velTarget = integratedSensorUnitsToMetersPerSecond(leader.getActiveTrajectoryVelocity());
  //     posTarget = integratedSensorUnitsToMeters(leader.getActiveTrajectoryPosition());
  //     appliedVoltage = leader.getMotorOutputVoltage();
  //     currentAmps = new double[] {leader.getStatorCurrent(), follower.getStatorCurrent()};
  //     tempCelcius = new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
  // }

  // public void setHeight(double heightMeters, boolean goSlow) {
  //     if (mLastCommandedPosition != heightMeters) {
  //         mLastCommandedPosition = heightMeters;
  //         if (Math.abs(getHeight() - heightMeters) < 0.25) {
  //             leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(3.0));
  //             leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(1.5));
  //         } else if (goSlow) {
  //             leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(6.0));
  //             leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(3.0));
  //         } else {
  //             leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(12.0));
  //             leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(3.0));
  //         }
  //     }
  //     double currentVelocityTarget = integratedSensorUnitsToMetersPerSecond(leader.getActiveTrajectoryVelocity());
  //     double elevatorKA = 0.0;
  //     if (currentVelocityTarget > lastVelocityTarget) {
  //         elevatorKA = kMaxAccel.get() * kForwardsKa.get();
  //     } else if (currentVelocityTarget < lastVelocityTarget) {
  //         elevatorKA = kMaxAccel.get() * -kBackwardsKa.get();
  //     }
  //     lastVelocityTarget = currentVelocityTarget;
  //     double elevatorKG = getHeight() < SECOND_STAGE_HEIGHT ? ELEVATOR_BELOW_STAGE1_KG : ELEVATOR_ABOVE_STAGE1_KG;
  //     double elevatorArbFF = MathUtil.clamp(elevatorKA + elevatorKG, -0.999, 0.999);
  //     leader.set(ControlMode.MotionMagic, metersToIntegratedSensorUnits(heightMeters), DemandType.ArbitraryFeedForward, elevatorArbFF);
  //     Logger.getInstance().recordOutput("Elevator/ArbFF", elevatorArbFF);
  // }

  // public void setPercent(double percent) {
  //     leader.set(ControlMode.PercentOutput, percent);
  // }

  // public void resetHeight(double newHeightMeters) {
  //     leader.setSelectedSensorPosition(newHeightMeters);
  // }

  // public void enableBrakeMode(boolean enable) {
  //     // leader.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  //     // follower.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);

  //     if(enable){
  //       leadConfig.idleMode(IdleMode.kBrake);
  //       followConfig.idleMode(IdleMode.kBrake);
  //     }else{
  //       leadConfig.idleMode(IdleMode.kCoast);
  //       followConfig.idleMode(IdleMode.kCoast);
  //     }
  // }

  // public void updateTunableNumbers() {
  //     if (kP.hasChanged(0)) {
  //         leader.config_kP(0, integratedSensorUnitsToMetersPerSecond(kP.get()) * 1023.0);
  //     }

  //     if (kD.hasChanged(0)) {
  //         leader.config_kD(0, integratedSensorUnitsToMetersPerSecond(kD.get()) * 1023.0);
  //     }

  //     if (kMaxVelocity.hasChanged(0)) {
  //         leader.config_kF(0, 1023.0/metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get()));
  //     }

  //     if (kMaxVelocity.hasChanged(0)) {
  //         leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get()));
  //     }

  //     if (kMaxAccel.hasChanged(0)) {
  //         leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(kMaxAccel.get()));
  //     }
  // }

  // /* converts integrated sensor units to meters */
  // private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
  //     return integratedSensorUnits * ((ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/2048.0);
  // }

  // /* converts meters to integrated sensor units */
  // private double metersToIntegratedSensorUnits(double meters) {
  //     return meters * (2048.0/(ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
  // }   

  // /* converts integrated sensor units to meters per second */
  // private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
  //     return integratedSensorUnits * ((ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/60.0);
  // }

  // /* converts meters per second to integrated sensor units */
  // private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
  //     return metersPerSecond * (60.0/(ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
  // }

  // /* returns the height of the climber in meters */
  // private double getHeight() {
  //     return integratedSensorUnitsToMeters(leader.getSelectedSensorPosition());
  // }
}
