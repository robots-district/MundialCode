package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ModeElevator;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.subsystems.ElevatorSystem;

public class moveElevatorCommandManual extends Command
{
    private ElevatorSystem elevatorSubsystem;
    private CommandXboxController joystick2;
    boolean finished = false;

    public moveElevatorCommandManual(CommandXboxController joystick2, ElevatorSystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick2 = joystick2;
        addRequirements(elevatorSubsystem);
    }
    @Override
    public void initialize(){
    }
    
    @Override
    public void execute()
    {
        if(Math.abs(joystick2.getLeftY()) > 0.1){
            CommandScheduler.getInstance().schedule(new SetMechanismState(ModeElevator.MANUAL));
            elevatorSubsystem.moveElevatorManual(joystick2.getLeftY());
        } 
        
    }
        
}
