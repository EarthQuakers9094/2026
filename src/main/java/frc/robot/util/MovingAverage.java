package frc.robot.util;

public class MovingAverage {

  private final double[] values;
  private int i = 0;
  private double currentSum = 0.0;

  public MovingAverage(int size) {
    values = new double[size];
  }

  public double addValue(double value) {
    currentSum -= values[i];
    currentSum += value;
    values[i] = value;

    i += 1;
    i %= values.length;

    return currentSum / values.length;
  }

  public double getStandardDeviation(double currentMean) {
    double sum = 0;
    for (int i = 0; i < values.length; i++) {
      sum += Math.abs(currentMean - values[i]);
    }
    return sum / values.length;
  }
}
