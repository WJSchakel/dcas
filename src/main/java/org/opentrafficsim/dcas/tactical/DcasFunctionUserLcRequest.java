package org.opentrafficsim.dcas.tactical;

import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * Function that translates a lane change request supplied by some other part of the model, in to the result of one of the DCAS
 * functions as used by the system.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasFunctionUserLcRequest implements BiFunction<TacticalContextEgo, Dcas, DcasFunctionResult>
{

    /** User lane change request supplier. */
    private final Supplier<LateralDirectionality> request;

    /**
     * Constructor.
     * @param request user lane change request supplier
     */
    public DcasFunctionUserLcRequest(final Supplier<LateralDirectionality> request)
    {
        this.request = request;
    }

    @Override
    public DcasFunctionResult apply(final TacticalContextEgo context, final Dcas dcas)
    {
        switch (this.request.get())
        {
            case NONE:
                return DcasFunctionResult.NONE;
            case LEFT:
                return DcasFunctionResult.CHANGE_LEFT;
            case RIGHT:
                return DcasFunctionResult.CHANGE_RIGHT;
            default:
                return DcasFunctionResult.NONE;
        }
    }

}
