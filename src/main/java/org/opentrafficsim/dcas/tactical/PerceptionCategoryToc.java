package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.gtu.perception.AbstractPerceptionCategory;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.gtu.perception.LanePerception;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelMental;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelTask;

/**
 * Perception category for perceiving a Transition Of Control request with delay.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class PerceptionCategoryToc extends AbstractPerceptionCategory<LaneBasedGtu, LanePerception>
{

    /**
     * Constructor.
     * @param perception perception
     */
    public PerceptionCategoryToc(final LanePerception perception)
    {
        super(perception);
    }

    /**
     * Returns Transition Of Control request.
     * @return Transition Of Control request
     */
    public boolean getTransitionOfControlRequest()
    {
        if (getPerception().getGtu().getTacticalPlanner() instanceof DcasTacticalPlanner dcasPlanner)
        {
            Duration when = getTimestamp();
            if (getPerception().getMental().isPresent())
            {
                if (getPerception().getMental().get() instanceof ChannelMental channelMental)
                {
                    when = when.minus(channelMental.getPerceptionDelay(ChannelTask.IN_VEHICLE));
                }
                else
                {
                    try
                    {
                        when = when.minus(getPerception().getGtu().getParameters().getParameter(ParameterTypes.TR));
                    }
                    catch (ParameterException ex)
                    {
                        // no reaction time or perception delay
                    }
                }
            }
            return dcasPlanner.getTransitionOfControlRequest(when);
        }
        return false;
    }

}
