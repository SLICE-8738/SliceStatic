// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



/* 
package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunElevator extends Command {
  private final Elevator elevator;
  /*
  private final Arm armSubsystem;
  private final Intake intakeSubsystem;
  */
  
  //private final CommandGenericHID operatorController;

  //private TrapezoidProfile.State state0 = new State(0, 0);
  //private TrapezoidProfile.State state1 = new State(35, 0);
  //private TrapezoidProfile.State state2 = new State(35, 0);
  //private TrapezoidProfile.State state3 = new State(70, 0);
  //private TrapezoidProfile.State state4 = new State(180, 0);

  //public int pos = -1;

  //public int grabPosition = 0;

  /* 
  public RunElevator(Elevator elevator, CommandGenericHID opController/*Intake iSubsystem, Arm aSubsystem) {
    //this.elevator = elevator;
    //operatorController = opController;
    /*
    intakeSubsystem = iSubsystem;
    armSubsystem = aSubsystem;
    addRequirements(elevator);
    
  }
*/
  //public Command prepToGrab(){
   // return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[1]))
                       //     .andThen(() -> armSubsystem.setPosition(state4))
                       //     .andThen(() -> armSubsystem.holdPosition(state4))
                       //     .andThen(() -> intakeSubsystem.stopIntake())
                       //     .andThen(() -> System.out.println("0: " + grabPosition));
  //}

  //public Command grab(){
   // return elevatorSubsystem.runOnce(() -> armSubsystem.holdPosition(state4))
                          //  .andThen(() -> intakeSubsystem.runIntake(intakeSubsystem.intakeSpeed))
                          //  .andThen(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[4]))
                          //  .andThen(() -> System.out.println("1: " + grabPosition));
  //}

  //public Command rest(){
   // return elevatorSubsystem.runOnce(() -> armSubsystem.setPosition(state0))
                     //       .andThen(() -> elevatorSubsystem.setPosition(0))
                     //       .andThen(() -> intakeSubsystem.stopIntake())
                     //       .andThen(() -> System.out.println("resting"))
                     //       .andThen(()-> grabPosition = 0);
  //}

  //public Command setPosition0(){
    //if(pos==-1){
    //  return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[0]))
                           //   .andThen(() -> armSubsystem.setPosition(state0))
                           //   .andThen(() -> armSubsystem.holdPosition(state0))
                           //   .andThen(() -> intakeSubsystem.stopIntake())
                           //   .andThen(() -> pos = 0);
    // }else if (pos == 0){
    //   return elevatorSubsystem.runOnce(() -> armSubsystem.setPosition(state0))
    //                         .andThen(() -> elevatorSubsystem.setPosition(0))
    //                           .andThen(() -> pos = -1);
    //} else {
   //   return elevatorSubsystem.runOnce(()->rest());
    //}
  //}

  //public Command setPosition1(){
    //if(pos==-1){
     // return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[1]))
                        //      .andThen(() -> armSubsystem.setPosition(state1))
                        //      .andThen(() -> armSubsystem.holdPosition(state1))
                        //    .andThen(() -> intakeSubsystem.stopIntake())
                         //     .andThen(() -> pos = 1);
    // }else{
    //   return elevatorSubsystem.runOnce(() -> pos = -1)
    //                           .andThen(() -> rest());
    // }
   // } else {
     // return elevatorSubsystem.runOnce(()->rest());
   // }
  //}

  //public Command setPosition2(){
    //if(pos==-1){
      //return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[2]))
                          //    .andThen(() -> armSubsystem.setPosition(state2))
                          //    .andThen(() -> armSubsystem.holdPosition(state2))
                          //  .andThen(() -> intakeSubsystem.stopIntake())
                          //    .andThen(() -> pos = 2);
    // }else{
    //   return elevatorSubsystem.runOnce(() -> pos = -1)
    //                           .andThen(() -> rest());
    // }
    //} else {
     // return elevatorSubsystem.runOnce(()->rest());
    //}
 // }

  //public Command setPosition3(){
    //if(pos==-1){
      //return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(elevatorSubsystem.heights[3]))
                    //          .andThen(() -> armSubsystem.setPosition(state3))
                    //          .andThen(() -> armSubsystem.holdPosition(state3))
                    //        .andThen(() -> intakeSubsystem.stopIntake())
                    //          .andThen(() -> pos = 3);
    // }else{
    //   return elevatorSubsystem.runOnce(() -> pos = -1)
    //                           .andThen(() -> rest());
    // }
   // } else {
      //return elevatorSubsystem.runOnce(()->rest());
   // }
  //}
  
//  public Command grabFunction = new SequentialCommandGroup(prepToGrab(), grab(), prepToGrab(), setPosition0(), rest());

  //public Command intakeGo(){
    //return intakeSubsystem.runOnce(() -> intakeSubsystem.runIntake(intakeSubsystem.intakeSpeed));
  //}

  //public Command intakeStop(){
   // return intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake());
  //}


  /** for elevator testing */
  //public Command run(){
     //return elevatorSubsystem.runOnce(() -> elevatorSubsystem.setPosition(24));
   //}
   //public Command stop(){
    //return elevatorSubsystem.runOnce(() -> elevatorSubsystem.stop());
  // }

  /*

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.manualElevator(operatorController.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.manualElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

*/