package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * DCAS as seen by internal functions.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public interface DcasSystemInterface
{

    /**
     * Sets a lowered speed for DCAS. This method can be called by system functions that want a lower speed than the user speed.
     * The minimum of all supplied values is used. If the user as set by the driver is lower, this method has no net effect.
     * @param loweredSystemSpeed lowered speed
     */
    void setLoweredSystemSpeed(Speed loweredSystemSpeed);

    /**
     * Sets an acceleration for DCAS. This method can be called by system functions that determine acceleration. The minimum of
     * all supplied values is used.
     * @param systemAcceleration system acceleration
     */
    void setSystemAcceleration(Acceleration systemAcceleration);

    /**
     * Returns maximum deceleration the system allows (a positive value).
     * @return maximum deceleration the system allows (a positive value)
     */
    Acceleration getMaximumDeceleration();

    /**
     * Returns car-following acceleration for the DCAS system.
     * @param context tactical context
     * @return car-following acceleration for the DCAS system
     */
    Acceleration getCarFollowingAcceleration(TacticalContextEgo context);

}
