// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  public SparkMax elevatorA = new SparkMax(11, MotorType.kBrushless);
  public SparkMax elevatorB = new SparkMax(12, MotorType.kBrushless);
  SparkMaxConfig configA = new SparkMaxConfig();
  SparkMaxConfig configB = new SparkMaxConfig();

  private RelativeEncoder encoder = elevatorA.getEncoder();
  EncoderConfig encoderConfig = new EncoderConfig();

  DigitalInput limitSwitch = new DigitalInput(0);

  private SparkClosedLoopController controller = elevatorA.getClosedLoopController();


  public double elevatorSpeed = 0.1; //untested!
  public double[] heights = {24.000, 31.875, 47.625, 72.000, 20.000}; //height in inches
  //                         L1      L2      L3      L4     grab
  public double currentPosition = 0;

  public double getPosition(){
    return encoder.getPosition();
  }

  public void setPosition(double p){
    controller.setReference(p, SparkMax.ControlType.kPosition);
  }

  
  public Elevator() {
    configA
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    // configA.encoder
    //   .positionConversionFactor(1000)
    //   .velocityConversionFactor(1000);
    configA.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(4.25, 0, 0.6);

    configB.apply(configA);
    elevatorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoderConfig
      .positionConversionFactor(1000);
  }

  //(s*5676/5)(currentPosition-p)
}