package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.subsystems.ElevatorSystem;

public class moveElevatorCommandAut extends Command
{
    private ElevatorSystem elevatorSubsystem;
    //private ElevatorState elevatorState;

    public moveElevatorCommandAut(ElevatorSystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute()
    {
        elevatorSubsystem.setPosition();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return elevatorSubsystem.atDesiredPosition();
    }
}
