package com.team2052.lib.requests;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveRequest {
    public int priority;
    public Translation2d robotTranslation; 
    public DriveRequest(int priority, Translation2d robotRelTranslation) {
        this.priority = priority;
        this.robotTranslation = robotRelTranslation;
    }
}
