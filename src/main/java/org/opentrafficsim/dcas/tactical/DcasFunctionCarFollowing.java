package org.opentrafficsim.dcas.tactical;

import java.util.function.BiFunction;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * Function that sets the car-following acceleration in DCAS.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasFunctionCarFollowing implements BiFunction<TacticalContextEgo, DcasSystemInterface, DcasFunctionResult>
{

    @Override
    public DcasFunctionResult apply(final TacticalContextEgo context, final DcasSystemInterface dcas)
    {

        Acceleration acceleration = dcas.getCarFollowingAcceleration(context);
        Acceleration maxB = dcas.getMaximumDeceleration().neg();

        if (acceleration.lt(maxB))
        {
            dcas.setSystemAcceleration(maxB);
            return DcasFunctionResult.TRANSITION_OF_CONTROL;
        }

        dcas.setSystemAcceleration(acceleration);
        return DcasFunctionResult.NONE;

    }

}
