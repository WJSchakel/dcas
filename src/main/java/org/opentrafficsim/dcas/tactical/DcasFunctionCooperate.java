package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.gtu.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.perception.object.PerceivedGtu;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * Applies cooperation as response to an indicator of a leader in any adjacent lane. Cooperation is applied by car-following the
 * leader with limited deceleration.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasFunctionCooperate implements DcasFunction
{

    @Override
    public DcasFunctionResult apply(final TacticalContextEgo context, final DcasSystemInterface dcas)
            throws OperationalPlanException, ParameterException
    {
        NeighborsPerception neighbors = context.getPerception().getPerceptionCategory(NeighborsPerception.class);
        cooperateWithLane(context, dcas, neighbors, RelativeLane.LEFT);
        cooperateWithLane(context, dcas, neighbors, RelativeLane.RIGHT);
        return DcasFunctionResult.NONE;
    }

    /**
     * Cooperates with a single lane.
     * @param context tactical context
     * @param dcas DCAS
     * @param neighbors neighbors perception
     * @param lane lane
     */
    private void cooperateWithLane(final TacticalContextEgo context, final DcasSystemInterface dcas,
            final NeighborsPerception neighbors, final RelativeLane lane)
    {
        PerceptionCollectable<PerceivedGtu, LaneBasedGtu> leaders = neighbors.getLeaders(lane);
        if (!leaders.isEmpty() && leaders.first().getSignals().isIndicatorOn(lane.getLateralDirectionality().flip()))
        {
            dcas.setSystemAcceleration(
                    Acceleration.max(dcas.getSetting(ParameterTypes.B).neg(), dcas.getCarFollowingAcceleration(context, lane)));
        }
    }

}
