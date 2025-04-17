// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.Constants.kDrivetrain.ArmConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.io.ObjectInputFilter.Config;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;



public class Arm extends SubsystemBase {
  private final ArmFeedforward ff =
    new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  public SparkMax arm = new SparkMax(20, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  private RelativeEncoder encoder = arm.getAlternateEncoder();

  public SparkClosedLoopController controller = arm.getClosedLoopController();

  public double[] angles    = {0, 35,            35,            50,   180}; //angles in degrees from vertical
//public double[] positions = {0, 0.48611111111, 0.48611111111, 1.25, 2.5}; //angles in number of rotations
  //                           L1 L2             L3             L4    grab
  public double currentPosition = 0;

  private TrapezoidProfile.State state0 = new State(0, 0);
  private TrapezoidProfile.State state1 = new State(35, 0);
  private TrapezoidProfile.State state2 = new State(35, 0);
  private TrapezoidProfile.State state3 = new State(70, 0);
  private TrapezoidProfile.State state4 = new State(180, 0);

  public Arm() {
    configureMotors();
  }

  public void configureMotors(){
    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    config.alternateEncoder
      .positionConversionFactor(72)
      .velocityConversionFactor(72)
      .countsPerRevolution(8192)
      .setSparkMaxDataPortConfig();
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0.1, 0, 0.01);
    arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
  }

  public void setPosition(TrapezoidProfile.State setpoint){
    // System.out.print("position: ");
    // System.out.println(setpoint.position);
    // System.out.print("encoder: ");
    // System.out.println(encoder.getPosition());

    double feedforward = ff.calculate(Units.degreesToRadians(setpoint.position)+Math.PI/2, setpoint.velocity); 

    controller.setReference(setpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  public void holdPosition(TrapezoidProfile.State setpoint){
    double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity); 
    controller.setReference(setpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  public double getPos(){
    return encoder.getPosition();
  }

  //Commands for use when constructing Autos
  public Command setPosition0Auto(){
    return runOnce(() -> holdPosition(state0));
  }

  public Command setPosition1Auto(){
    return runOnce(() -> holdPosition(state1));
  }

  public Command setPosition2Auto(){
    return runOnce(() -> holdPosition(state2));
  }

  public Command setPosition3Auto(){
    return runOnce(() -> holdPosition(state3));
  }
}