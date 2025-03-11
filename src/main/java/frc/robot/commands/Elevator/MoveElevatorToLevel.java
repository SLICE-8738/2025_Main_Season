package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ElevatorPositionSelector;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;

public class MoveElevatorToLevel extends Command {
    private final Elevator m_elevator;
    private final double m_elevatorThreshold;
    private double m_level;
    private LevelType m_levelType;

    /// true is upwards movement. false is downwards movement.
    private boolean movementDirection = false;

    public MoveElevatorToLevel(Elevator elevator, LevelType levelType) {
        addRequirements(elevator);

        m_elevator = elevator;
        m_elevatorThreshold = kElevator.THRESHOLD;
        m_levelType = levelType;
    }

    public void initialize() {
        switch(m_levelType){
            case SOURCE:
                m_level = Elevator.getSourceLevel().height;
                break;
            case CORAL:
                m_level = Elevator.getCoralLevel().height;
                break;
            case ALGAE:
                m_level = Elevator.getAlgaeLevel().height;
                break;
        }
        
        
        if (m_level > m_elevator.getPositions()[0]) {
            movementDirection = true;
        }
    }

    public void execute() {
        m_elevator.moveTo(m_level);
        SmartDashboard.putBoolean("Level Height", m_level  == Level.STOW.height);
        if (m_level == Level.STOW.height
                && Math.abs(m_elevator.getStatorCurrents()[0]) >= 30 && m_elevator.getPositions()[0] <= 0.1 && Math.abs(m_elevator.getVelocity()[0]) <= 0.01) {
            m_elevator.setEncoderPosition(0);
        }
    }

    public void end() {
        m_elevator.set(0);
    }

    public boolean isFinished() {
        boolean finished = false;
        if (m_elevator.atTarget(m_elevatorThreshold)) {
            finished = true;
        }
        // if (m_elevator.isAtTop() && movementDirection) {
        // finished = true;
        // }
        // if (m_elevator.isAtBottom() && !movementDirection) {
        // finished = true;
        // }

        return finished;
    }
}
