package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class GameState {
    public enum Phase {
        Transition,
        Active,
        Inactive,
        Auto,
        Endgame,
        NotInGame,
    }

    private static GameState instance;

    public static GameState getInstance() {
        if (instance == null) {
            instance = new GameState();
        }
        return instance;
    }

    private Phase currentPhase = Phase.NotInGame;

    public GameState() {
        if (DriverStation.getMatchType() != MatchType.None) {
            currentPhase = Phase.Auto;
        }
    }
    
}
