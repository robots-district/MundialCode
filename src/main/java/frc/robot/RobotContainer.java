// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberState;
import frc.robot.Constants.Controle;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;
import frc.robot.Constants.Tags;
import frc.robot.Constants.Timer;
import frc.robot.Constants.turboStates;
import frc.robot.commands.AutoCommands.SwerveLadoEsquerdaAuto;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetActions;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.commands.OdometriaAlinhamento.AlinhamentoOdometria;
import frc.robot.commands.Reef.Alinhamento3dDireita;
import frc.robot.commands.Reef.Alinhamento3dEsquerda;
import frc.robot.commands.Reef.AlinhamentoMeio;
import frc.robot.commands.Subsystem.AlgaeGet;
import frc.robot.commands.Subsystem.AlgaeShoot;
import frc.robot.commands.Subsystem.Intake;
import frc.robot.commands.Subsystem.ReShoot;
import frc.robot.commands.Subsystem.ReShootD;
import frc.robot.commands.Subsystem.Shoot;
import frc.robot.commands.Subsystem.ShootD;
import frc.robot.commands.Subsystem.cancelElevator;
import frc.robot.commands.Subsystem.moveElevatorCommand;
import frc.robot.commands.Subsystem.moveElevatorCommandAut;
import frc.robot.commands.Subsystem.moveElevatorCommandManual;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoDireita;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoEsquerda;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoFrente;
import frc.robot.commands.swervedrive.drivebase.Teleop;
import frc.robot.subsystems.AlgaeSystem;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.DeployerIntakeSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.PivoSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.fasterxml.jackson.core.util.RequestPayload;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  // Aqui iniciamos o swerve
  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private turboStates turbo = turboStates.marchaBaixa;
  boolean modoAlinhamento;

  boolean alianca = true;// true para vermelha, false para azul
  public final static ElevatorSystem elevatorSystem = new ElevatorSystem();
  public static final DeployerIntakeSystem deployerIntakeSystem = new DeployerIntakeSystem();
  public static final PivoSystem pivoSystem = new PivoSystem();
  public static final ClimberSystem climberSystem = new ClimberSystem();
  public static final AlgaeSystem algaeSystem = new AlgaeSystem();

  public static final Led led = new Led();
  
  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<Integer> pipelineChooser = new SendableChooser<>();
  public moveElevatorCommand moveElevatorCommand;
  GetSensor getSensor;

  //Controle piloto
  CommandXboxController controleXbox = new CommandXboxController(Controle.xboxControle);

  CommandXboxController buttonJoy = new CommandXboxController(Controle.buttonBox);
  
  public CommandXboxController joymanual = new CommandXboxController(Controle.joyManual);

  public RobotContainer() {
    // Definimos o comando padrão como a tração
    swerve.setDefaultCommand(new Teleop(swerve,
      () -> -MathUtil.applyDeadband(controleXbox.getLeftY() * turbo.velocidade, Constants.Controle.DEADBAND),
      () -> -MathUtil.applyDeadband(controleXbox.getLeftX() * turbo.velocidade, Constants.Controle.DEADBAND) ,
      () -> -MathUtil.applyDeadband(controleXbox.getRightX() * turbo.velocidade / 1.7, Constants.Controle.DEADBAND)));

    
    //  NamedCommands.registerCommand("Stop", 
    //     new InstantCommand(() -> swerve.stop(), swerve)
    // );
    //  NamedCommands.registerCommand("Encoder", 
    //     new InstantCommand(() -> moveElevatorCommand.ConferirEnconder(), elevatorSystem)
    // );

    
    // NamedCommands.registerCommand("L1", 
    // new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L2", 
    // new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L3", 
    //  new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L4", 
    //  new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("Repouso", 
    //  new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("Pontuar", 
    //  new Shoot(deployerIntakeSystem));
    // NamedCommands.registerCommand("Recolher",
    //  new SequentialCommandGroup(new Intake(deployerIntakeSystem), new ReShoot(deployerIntakeSystem)));
    
    
      NamedCommands.registerCommand("AlinharD17", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.frenteDireita, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE17", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.frenteDireita, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE18", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.Frente, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD18", 
      new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.Frente, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD19", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.FrenteEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE19", 
      new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.FrenteEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE20", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.trasEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD20", 
      new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.trasEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD21", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.tras, Timer.Autonomous)
      );
    
      NamedCommands.registerCommand("AlinharE21", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.tras, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE22", 
    new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.trasDireita, Timer.Autonomous)
);
 
     
      NamedCommands.registerCommand("AlinharD22", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.trasDireita, Timer.Autonomous)
      );

      NamedCommands.registerCommand("L1", 
      new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L2", 
      new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L3", 
       new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L4", 
       new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("Repouso", 
       new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("Pontuar", 
       new Shoot(deployerIntakeSystem));
      NamedCommands.registerCommand("Recolher",
       new SequentialCommandGroup(new Intake(deployerIntakeSystem)));
      
      
      autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomo", autoChooser);
    




    if(!Robot.isReal()){
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    // Configure the trigger bQindings
    configureBindings();
    configurePipelineSelector();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    

     controleXbox.y().whileTrue(new SwerveLadoFrente(controleXbox, swerve));
     controleXbox.x().whileTrue(new SwerveLadoDireita(controleXbox, swerve));    
     controleXbox.b().whileTrue(new SwerveLadoEsquerda(controleXbox, swerve));
    //controleXbox.a().whileTrue(new SwerveAutoAlinhamentoCompleto(swerve, controleXbox));

    controleXbox.leftBumper().onTrue(new InstantCommand(() -> turbo = turboStates.marchaBaixa));
    controleXbox.rightBumper().onTrue(new InstantCommand(() -> turbo = turboStates.marchaAlta));

    controleXbox.start().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    
    controleXbox.povUp().onTrue(new InstantCommand(() -> swerve.visao = false));
    controleXbox.povDown().onTrue(new InstantCommand(() -> swerve.visao = true));

    //controleXbox.y().onTrue(new InstantCommand(() -> modoAlinhamento = false));  
    //controleXbox.a().onTrue(new InstantCommand(() -> modoAlinhamento = true));

    joymanual.axisMagnitudeGreaterThan(1,0.1).whileTrue(new moveElevatorCommandManual(joymanual, elevatorSystem)).onFalse(new cancelElevator(elevatorSystem));

    // joymanual.rightTrigger().whileTrue(new AlgaeGet(algaeSystem));
    // joymanual.leftTrigger().whileTrue(new AlgaeShoot(algaeSystem));

    joymanual.rightTrigger().whileTrue(new AlgaeShoot(algaeSystem));
    joymanual.leftTrigger().whileTrue(new AlgaeGet(algaeSystem));
    // joymanual.leftTrigger().onTrue(new Intake(deployerIntakeSystem));

    joymanual.button(5).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    joymanual.button(1).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.ALGAE1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));
    joymanual.button(4).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.ALGAE2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));
    joymanual.button(2).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.NET).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));

    joymanual.povUp().onTrue(new SetMechanismState(PivoState.OPEN));
    joymanual.povDown().onTrue(new SetMechanismState(PivoState.CLOSED));

    joymanual.button(6).whileTrue(new ReShootD(deployerIntakeSystem));

    joymanual.button(8).onTrue(new SetActions().ClimbPosition());

    controleXbox.rightTrigger().whileTrue(new SetMechanismState(ClimberState.OPENING)).onFalse(new SetMechanismState(ClimberState.STOPPED));
    // controleXbox.leftTrigger().whileTrue(new SetMechanismState(ClimberState.CLOSING)).onFalse(new SetMechanismState(ClimberState.STOPPED));

    buttonJoy.button(15).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(13).onTrue(new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(11).onTrue(new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(9).onTrue(new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(6).onTrue(new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));

    // buttonJoy.button(4).onTrue(new Intake(deployerIntakeSystem));
    buttonJoy.button(4).onTrue(new SequentialCommandGroup(new Intake(deployerIntakeSystem), new ReShoot(deployerIntakeSystem)));
    buttonJoy.button(1).whileTrue(new ShootD(deployerIntakeSystem));
    buttonJoy.button(7).onTrue(new SetActions().ClimbPosition());
    ButtonBoxPP();

    controleXbox.a().onTrue(new InstantCommand(() -> {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Tags.allTags);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Tags.allTags);
    }));
  }
  
  // funcao que retorna o autonomous
  
  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

   private void configurePipelineSelector() {
        // Adiciona opções de pipelines
        pipelineChooser.setDefaultOption("Pipeline 0", 0);
        pipelineChooser.addOption("Pipeline 1", 1);
        pipelineChooser.addOption("Pipeline 2", 2);
        pipelineChooser.addOption("Pipeline 3", 3);
        pipelineChooser.addOption("Pipeline 4", 4);
        pipelineChooser.addOption("Pipeline 5", 5);
        pipelineChooser.addOption("Pipeline 6", 6);

        // Envia o chooser para a SmartDashboard
        SmartDashboard.putData("Pipeline Selector", pipelineChooser);
    }

    public int getSelectedPipeline() {
      return pipelineChooser.getSelected();
  }

  public void ButtonBoxPP(){
    
    
    //Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    //boolean isRedAlliance = (alliance == Alliance.Red);
    boolean isRedAlliance = alianca;

    System.out.println("Estamos na aliança " + (isRedAlliance ? "vermelha" : "azul") + "!");
    // Frente (Direita)
    buttonJoy.button(16).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 7 : 18, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0 , 0, true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente (Esquerda)
    buttonJoy.button(14).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 7 : 18, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0, true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Frente Direita (Direita)
    buttonJoy.button(12).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 8 : 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente Direita (Esquerda)
    buttonJoy.button(10).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 8 : 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Frente Esquerda (Direita)
    buttonJoy.button(18).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 6 : 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente Esquerda (Esquerda)
    buttonJoy.button(17).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 6 : 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp),
        () -> modoAlinhamento
    ));


    //Tras Direita (Direita)
    buttonJoy.button(8).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Direita (Esquerda)
    buttonJoy.button(5).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Esquerda (Direita)
    buttonJoy.button(20).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Esquerda (Esquerda)
    buttonJoy.button(19).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Tras (Direita)
    buttonJoy.button(2).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras (Esquerda)
    buttonJoy.button(21).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp),
        () -> modoAlinhamento
    ));
}
    public Command Autonomo() {
        // Aqui retornamos o comando que está no selecionador
        return autoChooser.getSelected();   
      }

  public Command Direita(){ 
    boolean isRedAlliance = alianca; // true para vermelho, e false para azul


   return new SequentialCommandGroup(

  // new ParallelCommandGroup(
       new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.4,0,true, false ),
       //new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)))

   //),
          
              
        //alinhar 
        
            
            new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
            new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.Autonomous),
           
        new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
            
        //new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0,0,true, false ),
     
    //   pontuar
      
      new Shoot(deployerIntakeSystem),
      //recolher
       
          new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 2 : 12, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.04, 2.3, true, false),
      
        new Intake(deployerIntakeSystem),
        new ReShoot(deployerIntakeSystem),
            
    

              
        // new Intake(deployerIntakeSystem),
        // new ReShoot(deployerIntakeSystem),

    //   new ParallelCommandGroup(
          new AlinhamentoOdometria(swerve, isRedAlliance ?  8: 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.35, 0, false, true ),
         // new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)))
        
      
          
              
        //    new Intake(deployerIntakeSystem),
        //    new ReShoot(deployerIntakeSystem),       
              
        //    // alinhar
        new ParallelCommandGroup(
            new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
            new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.Autonomous)
            
        ),
        new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
          
              
    //   //pontuar
      
      
      new Shoot(deployerIntakeSystem),

      
      new ParallelCommandGroup(

          new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
          new AlinhamentoOdometria(swerve, isRedAlliance ? 2 : 12, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.48, 1.4, true, false),
          
          new SequentialCommandGroup(
              new Intake(deployerIntakeSystem),
              new ReShoot(deployerIntakeSystem)
            
          )
          ),
              
              
        // new ParallelCommandGroup(

            new AlinhamentoOdometria(swerve, isRedAlliance ?  8: 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0 , false, true),
            //new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)))
        // ),      
                 
        
    //   new Intake(deployerIntakeSystem),
    //   new ReShoot(deployerIntakeSystem),

    new ParallelCommandGroup(
        new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.Autonomous)
        
    ),
    new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
          
              
     // new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
       new Shoot(deployerIntakeSystem)
       
      // new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)))
      );
  }


  public Command Esquerda(){
    boolean isRedAlliance = alianca; // true para vermelho, e false para azul
//Alinhar telemetria com L2
   return new SequentialCommandGroup(
    
    //new ParallelCommandGroup(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.4,0,true, false),
        //new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)))

    //),
        
            
            

            

    
            //Alinhar Odometria com L4
            
            new ParallelCommandGroup(
                
                new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
                new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.Autonomous)
                ),    
                    new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
           
            
            
            // pontuar
            new Shoot(deployerIntakeSystem),


            //recolher o elevador e ir para a Coral Station
           
                new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
                new AlinhamentoOdometria(swerve, isRedAlliance ? 1 : 13, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.025, 2.3, true, false),
           
                new Intake(deployerIntakeSystem),
                new ReShoot(deployerIntakeSystem),
                
            
             
                
                    


        //Pegar o coral
                    
            //Ir parea o reef telemetria
            new AlinhamentoOdometria(swerve, isRedAlliance ?  6: 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.35, 0, false, true),
                       
                    
                    
            // alinhar Odemtria com L4
            new ParallelCommandGroup(
                new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem)),
                new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.Autonomous))
                
                ),
                new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
          
              

              //pontuar             
              new Shoot(deployerIntakeSystem),


              // Ir para a Coral Station descendo pro Coral
        new ParallelCommandGroup(
            new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
            new AlinhamentoOdometria(swerve, isRedAlliance ? 1 : 13, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.025, 2.3, true, false),
        new SequentialCommandGroup(
            new Intake(deployerIntakeSystem),
            new ReShoot(deployerIntakeSystem)

        )    
        ),
   

    
     //Pegar o Coral


    // Ir para o reef
         new AlinhamentoOdometria(swerve, isRedAlliance ?  6: 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, true),


      // Alinhar Odometria Com l4   
     
      new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.Autonomous),
      new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
        
        
    
        
        //pontuar
        new Shoot(deployerIntakeSystem)
   );
        
        
        
        
   
    }
    public Command Meio(){
        boolean isRedAlliance = alianca; // true para vermelho, e false para azul
        
        return new SequentialCommandGroup(
            //alinhar 
            
            new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.1, 0, true, false),
            new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.Autonomous),
            new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommandAut(elevatorSystem))),
      // pontuar
      new Shoot(deployerIntakeSystem)
      //recolher
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC))
    

      );
  }

  public Command MeioAlga(){
    boolean isRedAlliance = alianca;

    return new SequentialCommandGroup(
        //alinhar 
        
      //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, true, false),
      new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.Autonomous),
      // ir para o meio (direita) ou esquerda...
      // ir para tras um pouco, descer
      
      new Shoot(deployerIntakeSystem),
      
      new SwerveLadoEsquerdaAuto(controleXbox, swerve, 0.7),
      
      new AlinhamentoOdometria(swerve, isRedAlliance ? 5 : 14, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0.7, 0, false, false),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20 , buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, true),
      new AlinhamentoMeio(),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 5 : 14 , buttonJoy, Constants.AlinhamentoOdometria.graus180, 0.7, 1.35, true, false)
      //recolher
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC))
    

      );
      
  }
}