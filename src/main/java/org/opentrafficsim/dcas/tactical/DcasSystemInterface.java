package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
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
     * Returns setting value.
     * @param setting setting
     * @return setting value
     * @param <T> value type
     */
    <T> T getSetting(ParameterType<T> setting);

    /**
     * Sets a lowered speed for DCAS. This method can be called by system functions that want a lower speed than the user speed.
     * The minimum of all supplied values is used. If the user speed is lower, this method has no net effect.
     * @param loweredSystemSpeed lowered speed
     */
    void setLoweredSystemSpeed(Speed loweredSystemSpeed);

    /**
     * Returns car-following acceleration for the DCAS system. This method returns decelerations that are not limited to any
     * particular value. Depending on the context, the caller must recognize decelerations beyond relevant thresholds and adapt
     * accordingly. This method can be used by any internal DCAS function to determine the system appropriate car-following
     * acceleration for the given lane. Each function is expected to perform their own logic to the resulting acceleration, such
     * as applying a deceleration limit, and forward the result to {@link #setSystemAcceleration}. Then, the minimum
     * acceleration supplied by the functions applies.
     * @param context tactical context
     * @param lane lane at which to follow the leader (i.e. for regular car-following or for synchronization)
     * @return car-following acceleration for the DCAS system
     */
    Acceleration getCarFollowingAcceleration(TacticalContextEgo context, RelativeLane lane);

    /**
     * Sets an acceleration for DCAS. This method can be called by system functions that determine acceleration. The minimum of
     * all supplied values is used.
     * @param systemAcceleration system acceleration
     */
    void setSystemAcceleration(Acceleration systemAcceleration);

}
