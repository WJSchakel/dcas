package org.opentrafficsim.dcas.tactical;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.ControlType;
import org.opentrafficsim.dcas.tactical.MatrixPerceptionCategory.PerceivedMatrixSign;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.perception.object.PerceivedGtu;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * DCAS function that causes deceleration at maximum comfort level regarding dynamic matrix signs, or lowering of set system
 * speed for static matrix signs.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasFunctionMatrix implements DcasFunction
{

    @Override
    public DcasFunctionResult apply(final TacticalContextEgo context, final DcasSystemInterface dcas)
            throws OperationalPlanException, ParameterException
    {
        RelativeLane lane = RelativeLane.CURRENT;
        var signs = context.getPerception().getPerceptionCategory(MatrixPerception.class).getMatrixSigns(lane);
        var leaders = context.getPerception().getPerceptionCategory(NeighborsPerception.class).getLeaders(lane);
        for (PerceivedMatrixSign sign : signs)
        {
            if (sign.matrixState().isSpeedLimit())
            {
                if (ControlType.DYNAMIC.equals(sign.controlType()))
                {
                    // dynamically controlled matrix sign: reduce maximum comfortable deceleration based on ego speed
                    if (leaders.isEmpty())
                    {
                        // at sensor range
                        dcas.setSystemAcceleration(dcas.getAcceleration(context, dcas.getSetting(Dcas.SENSOR_RANGE),
                                sign.matrixState().getSpeed(), 1.0));
                        return DcasFunctionResult.NONE;
                    }
                    else
                    {
                        // follow leader
                        PerceivedGtu leader = leaders.first();
                        dcas.setSystemAcceleration(dcas.getAcceleration(context, leader.getDistance(), leader.getSpeed(), 1.0));
                        return DcasFunctionResult.NONE;
                    }
                }
                else
                {
                    // statically controlled matrix sign: apply custom speed limit
                    dcas.setLoweredSystemSpeed(sign.matrixState().getSpeed());
                    return DcasFunctionResult.NONE;
                }
            }
        }
        return DcasFunctionResult.NONE;
    }

}
