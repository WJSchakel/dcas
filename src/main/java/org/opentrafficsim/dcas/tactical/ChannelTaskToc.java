package org.opentrafficsim.dcas.tactical;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.constraint.DualBound;
import org.opentrafficsim.road.gtu.perception.LanePerception;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelTask;

/**
 * Task that reflects mental task demand due to a Transition Of Control request. After the request the task demand decays
 * exponentially based on parameter TAU (relaxation time, initially for headway relaxation after a lane change).
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class ChannelTaskToc implements ChannelTask
{

    /** Level of task demand during Transition Of Control. */
    public static final ParameterTypeDouble TD_TOC =
            new ParameterTypeDouble("TD_TOC", "Task demand during transition of control.", 0.5, DualBound.UNITINTERVAL);

    /** Cached task demand. */
    private double td;

    /** Last time task demand was calculated. */
    private double tLast = -1.0;

    @Override
    public double getTaskDemand(final LanePerception perception) throws ParameterException
    {
        double tdNew;
        if (perception.getGtu().getTacticalPlanner() instanceof DcasTacticalPlanner dcasPlanner
                && dcasPlanner.getTransitionOfControlRequest(perception.getGtu().getSimulator().getSimulatorTime()))
        {
            tdNew = perception.getGtu().getParameters().getParameter(TD_TOC);
        }
        else
        {
            tdNew = 0.0;
        }
        if (tdNew > this.td || this.tLast < 0.0)
        {
            this.td = tdNew;
            this.tLast = perception.getGtu().getSimulator().getSimulatorTime().si;
        }
        else
        {
            // exponential decay
            double t = perception.getGtu().getSimulator().getSimulatorTime().si;
            double ratio =
                    (t - this.tLast) / perception.getGtu().getParameters().getParameter(DcasTacticalPlanner.TAU_STIM).si;
            this.tLast = t;
            this.td = ratio > 1.0 ? tdNew : ratio * tdNew + (1.0 - ratio) * this.td;
        }
        return this.td;
    }

    @Override
    public double getTaskDemand()
    {
        return this.td;
    }

    @Override
    public String getId()
    {
        return "TOC";
    }

    @Override
    public Object getChannel()
    {
        return IN_VEHICLE;
    }

}
