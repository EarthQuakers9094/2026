package frc.robot;

public class SimState {

  private static SimState instance;

  public static SimState getInstance() {
    if (instance == null) {
      instance = new SimState();
    }
    return instance;
  }

  public boolean isIndexing = false;

  public SimState() {
    this.isIndexing = false;
  }
}
