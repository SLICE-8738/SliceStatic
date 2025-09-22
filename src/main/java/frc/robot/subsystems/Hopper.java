// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hopper extends SubsystemBase {
  public SparkMax hopperA = new SparkMax(9, MotorType.kBrushless);
  public SparkMax hopperB = new SparkMax(10, MotorType.kBrushless);
  
  public double hopperSpeed = 0.3; //from 2024 code, untested
  public double reverseHopperSpeed = 0.90; //from 2024 code, untested

  public Hopper() {}

  public Command runLeft(double speed){
    return runOnce(() -> hopperA.set(speed));
  }

  public Command reverseLeft(double speed){
    return runOnce(() -> hopperA.set(-speed));
  }

  public Command runRight(double speed){
    return runOnce(() -> hopperB.set(-speed));
  }

  public Command reverseRight(double speed){
    return runOnce(() -> hopperB.set(speed));
  }
  

  //Commands for use when constructing Autos
  public Command runHopperAuto(){
    return runOnce(() -> hopperA.set(hopperSpeed)).andThen(this.runOnce(() -> hopperB.set(-hopperSpeed)));
  }
  
  public Command stopHopperAuto(){
    return runOnce(() -> hopperA.set(0)).andThen(this.runOnce(() -> hopperB.set(0)));
  }
}

*/