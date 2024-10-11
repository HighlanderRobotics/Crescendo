package frc.robot.utils;

public class Battery {
    private String name;
    private double startingVoltage;
    private double endingVoltage;
    private boolean isFMS;
    private double timeConnected;
    
    public Battery(String name, double startingVoltage, boolean isFMS) {
        this.name = name;
        this.startingVoltage = startingVoltage;
        this.isFMS = isFMS;
    }

    public String getName() {
        return name;
    }

    public double getStartingVoltage() {
        return startingVoltage;
    }

    public double getEndingVoltage() {
        return endingVoltage;
    }

    public boolean isFMS() {
        return isFMS;
    }

    public double getTimeConnected() {
        return timeConnected;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setStartingVoltage(double startingVoltage) {
        this.startingVoltage = startingVoltage;
    }

    public void setEndingVoltage(double endingVoltage) {
        this.endingVoltage = endingVoltage;
    }

    public void setFMS(boolean isFMS) {
        this.isFMS = isFMS;
    }

    public void setTimeConnected(double timeConnected) {
        this.timeConnected = timeConnected;
    }


}
