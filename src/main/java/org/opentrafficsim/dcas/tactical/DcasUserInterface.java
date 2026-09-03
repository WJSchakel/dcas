package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.road.gtu.operational.SimpleOperationalPlan;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * DCAS as seen by the tactical planner.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public interface DcasUserInterface
{

    /**
     * Sets the enabled status of DCAS. This is controlled by the human part of the tactical model.
     * @param enabled enabled status
     */
    void setEnabled(boolean enabled);

    /**
     * Returns enabled status of DCAS.
     * @return enabled status of DCAS
     */
    boolean isEnabled();

    /**
     * Sets the user speed for DCAS. This method should be called by the human side of the tactical planner. The system may
     * decide to drive slower than this value.
     * @param userSpeed user speed
     */
    void setUserSpeed(Speed userSpeed);

    /**
     * Returns last acceleration.
     * @return last acceleration
     */
    Acceleration getAcceleration();

    /**
     * Returns simple operational plan (acceleration and lane change decision) of DCAS.
     * @param context tactical context
     * @return simple operational plan (acceleration and lane change decision) of DCAS
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    SimpleOperationalPlan getSimplePlan(TacticalContextEgo context) throws OperationalPlanException, ParameterException;

    /**
     * Returns DCAS state.
     * @return DCAS state
     */
    DcasState getState();

    /**
     * DCAS state.
     */
    enum DcasState
    {
        /** Disabled. */
        OFF,

        /** Enabled and in normal operation. */
        ON,

        /** Requesting Transition Of Control. */
        TOC,

        /** Executing Minimum Risk Maneuver. */
        MRM;
    }

}
