// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import frc.robot.utils.logging.TalonFXLogger.TalonFXLog;

/** Class to periodically check all talonfx logs for errors. */
public class TalonFXFaultManager {
    private static TalonFXFaultManager instance;

    private Map<String, TalonFXLog> logs = new HashMap<String, TalonFXLog>(20);
    private boolean ok = true;
    private boolean stickyOk = true;

    private TalonFXFaultManager() {}

    public static TalonFXFaultManager getInstance() {
        if (instance == null) instance = new TalonFXFaultManager();
        return instance;
    }

    public void addLog(String name, TalonFXLog log) {
        logs.put(name, log);
    }

    public void periodic() {
        ok = true;
        List<String> faults = new ArrayList<>();
        for (var log : logs.entrySet()) {
            // Check license faults
            if (log.getValue().licenseFault) {
                ok = false;
                faults.add(log.getKey() + " License Fault");
            }
        }
        stickyOk |= ok;
        Logger.recordOutput("Faults/TalonFX Fault Status", ok);
        Logger.recordOutput("Faults/TalonFX Sticky Fault Status", stickyOk);
        Logger.recordOutput("Faults/TalonFX Faults", faults.stream().reduce("", (a, s) -> a + s));
        logs.clear();
    }

    /** Returns if a fault has happened this loop. */
    public boolean isOk() {
        return ok;
    }

    /** Returns if a fault has happened since boot. */
    public boolean isOkSticky() {
        return stickyOk;
    }
}
