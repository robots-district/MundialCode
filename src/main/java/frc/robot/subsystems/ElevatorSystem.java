// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;

import frc.robot.Constants.ModeElevator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSystem extends SubsystemBase {
  public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.elevatorLeftMotorID);
  public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.elevatorRightMotorID);

  MotionMagicExpoVoltage motionRequest;

  public ElevatorState stateElevator = ElevatorState.CORAL;
  public ModeElevator modeElevator = ModeElevator.MANUAL;

  public boolean resetRepouso = false;

  public boolean gravity = false;

  public ElevatorSystem() {

    elevatorLeftMotor.getConfigurator().apply(Constants.ELEVATOR_CONFIG);
    elevatorRightMotor.getConfigurator().apply(Constants.ELEVATOR_CONFIG);

    motionRequest = new MotionMagicExpoVoltage(0);

    elevatorRightMotor.setControl(new Follower(Constants.ElevatorConstants.elevatorLeftMotorID, true));

    zeroElevator();
  }



  @Override
  public void periodic() {
    
  
    // Primeira coisa a se testar, objetivo e descobrir a tensão minima que o elevador começa a se mover
        // elevatorLeftMotor.set(0.1);
        // elevatorRightMotor.set(0.1);

        //segunda coisa descobrir O KV

        // System.out.println("Velocidade: " + getTargetAcceleration());

        // Enviar dados para o SmartDashboard para monitoramento
        SmartDashboard.putNumber("EncoderElevator esquerda", elevatorLeftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("EncoderElevator direita", elevatorRightMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("EncoderElevatorRIGHT", elevatorRightMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("EncoderElevatorRIGHT", elevatorRightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Posição Alvo", stateElevator.position);

        // System.out.println(elevatorLeftMotor.getMotorVoltage());
        // SmartDashboard.putNumber("Feedforward", feedforwardOutput);
        // SmartDashboard.putNumber("PID Output", pidOutput);
        // SmartDashboard.putNumber("Saída Final", totalOutput);

        // elevatorRightMotor.setControl(new Follower(3, true));

        SmartDashboard.putNumber("Corrente Motor Esquerda", elevatorLeftMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Corrente Motor Direita", elevatorRightMotor.getStatorCurrent().getValueAsDouble());
        
        
        }  

  public void moveElevatorManual(double speed){
    SmartDashboard.putNumber("Speed", speed);
    if(modeElevator.mode && getElevatorEncoderPosition() <= Constants.ElevatorState.CORAL.position && speed <= 0){
      elevatorLeftMotor.set(Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() >= Constants.ElevatorState.L4.position && speed > 0){
      elevatorLeftMotor.set(-Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() > Constants.ElevatorState.CORAL.position && getElevatorEncoderPosition() < Constants.ElevatorState.L4.position){
      if(speed <= 0){
        elevatorLeftMotor.set(Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
      } else{
        elevatorLeftMotor.set(-Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
      }
    } else if(modeElevator.mode){
      stopElevator();
    }
  }

  public void setPosition() {
    elevatorLeftMotor.setControl(motionRequest.withPosition(stateElevator.position));
  }

  public void stopElevator(){
    elevatorLeftMotor.stopMotor();
  }

  public boolean atDesiredPosition() {
    return isAtSetPointWithTolerance(stateElevator.position);
  }

  public boolean isAtSetPointWithTolerance(Double position) {
    return (getElevatorEncoderPosition() > stateElevator.position - 1.0) &&
        getElevatorEncoderPosition() < stateElevator.position + 1.0;
  }

  public double getElevatorEncoderPosition() {
    return elevatorLeftMotor.getPosition().getValueAsDouble();
    
  }
  public double getElevatorEncoderPositionRight() {
    return elevatorRightMotor.getPosition().getValueAsDouble();
    
  }

  public void setElevatorEncoderPosition(double position) {
    elevatorLeftMotor.setPosition(position);
  }

  public void zeroElevator(){
    elevatorLeftMotor.setPosition(0);
  }

  public void SetCurrentStateElevator(ElevatorState state){
    this.stateElevator = state;
  }

  public void SetCurrentModeElevator(ModeElevator state){
    this.modeElevator = state;
  }

  public void SetGravity(Boolean gravity){
    this.gravity = gravity;
  }

}
