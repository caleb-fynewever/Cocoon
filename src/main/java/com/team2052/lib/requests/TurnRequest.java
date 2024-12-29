package com.team2052.lib.requests;

public class TurnRequest {
  public int priority;
  public double rotationRadPerSecond;

  public TurnRequest(int priority, double rotationRadPerSecond) {
    this.priority = priority;
    this.rotationRadPerSecond = rotationRadPerSecond;
  }
}
