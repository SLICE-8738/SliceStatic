// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.io.ObjectInputFilter.Config;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;



public class Arm extends SubsystemBase {
  
  //public SparkMax arm = new SparkMax(20, MotorType.kBrushless);
  //public RelativeEncoder encoder = arm.getAlternateEncoder(/*8192*/); //
  //public SparkClosedLoopController controller = arm.getClosedLoopController();

  public double[] angles    = {0, 35,            35,            90,   180}; //angles in degrees from vertical
  public double[] positions = {0, 0.48611111111, 0.48611111111, 1.25, 2.5}; //angles in number of rotations
  //                           L1 L2             L3             L4    grab
  public double currentPosition;

  public Arm() {
    // SparkMaxConfig config = new SparkMaxConfig();

    // config
    //   .inverted(true)
    //   .idleMode(IdleMode.kBrake);
    // config.encoder
    //   .positionConversionFactor(1000)
    //   .velocityConversionFactor(1000);
    // config.closedLoop
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   .pid(4.25, 0, 0.6);

    // arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // public double getPosition(){
  //   return encoder.getPosition();
  // }

  public void setPosition(double p){
    //controller.setReference(p, SparkMax.ControlType.kPosition);
  }
}
