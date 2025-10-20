// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeSystem extends SubsystemBase {
  public SparkMax algaeMotor = new SparkMax(Constants.AlgaeConstants.algaeMotorID, MotorType.kBrushless);

  SparkMaxConfig configSparkMotor = new SparkMaxConfig();

  public AlgaeSystem() {
    configSparkMotor
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    configSparkMotor.smartCurrentLimit(50);

    algaeMotor.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    SmartDashboard.putNumber("Current Motor ALGAE", algaeMotor.getOutputCurrent());
  }

}
