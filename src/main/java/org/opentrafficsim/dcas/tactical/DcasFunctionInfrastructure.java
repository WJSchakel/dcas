package org.opentrafficsim.dcas.tactical;

import java.util.function.BiFunction;

import org.djunits.value.vdouble.scalar.Length;
import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.DirectInfrastructurePerception;
import org.opentrafficsim.road.gtu.perception.categories.InfrastructurePerception;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;
import org.opentrafficsim.road.network.LaneChangeInfo;

/**
 * DCAS function that will perform lane changes for infrastructure. If the lane change is not executed by the system, this
 * function escalates to a Transition Of Control request, or ultimately a Minimum Risk Maneuver.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasFunctionInfrastructure implements BiFunction<TacticalContextEgo, DcasSystemInterface, DcasFunctionResult>
{

    /** Remaining length per lane change below which DCAS attempts a lane change. */
    public static final ParameterTypeLength X_LC =
            new ParameterTypeLength("xLC", "Remaining length per lane change below which DCAS attempts a lane change",
                    Length.ofSI(500.0), NumericConstraint.POSITIVE);

    /** Remaining length per lane change below which DCAS requests Transition Of Control. */
    public static final ParameterTypeLength X_TOC =
            new ParameterTypeLength("xToc", "Remaining length per lane change below which DCAS requests Transition Of Control",
                    Length.ofSI(300.0), NumericConstraint.POSITIVE);

    /** Remaining length per lane change below which DCAS performs Minimum Risk Maneuver. */
    public static final ParameterTypeLength X_MRM =
            new ParameterTypeLength("xMrm", "Remaining length per lane change below which DCAS performs Minimum Risk Maneuver",
                    Length.ofSI(50.0), NumericConstraint.POSITIVE);

    @Override
    public DcasFunctionResult apply(final TacticalContextEgo context, final DcasSystemInterface dcas)
    {
        try
        {
            LaneChangeInfo lcInfo = getLaneChangeInfo(context);
            if (lcInfo == null)
            {
                return DcasFunctionResult.NONE;
            }
            Length distPerLc = lcInfo.remainingDistance().divide(lcInfo.numberOfLaneChanges());
            if (distPerLc.lt(context.getParameters().getParameter(X_MRM)))
            {
                return DcasFunctionResult.SHOULDER;
            }
            if (distPerLc.lt(context.getParameters().getParameter(X_TOC)))
            {
                return DcasFunctionResult.TRANSITION_OF_CONTROL;
            }
            if (distPerLc.lt(context.getParameters().getParameter(X_LC)))
            {
                LateralDirectionality dir = lcInfo.lateralDirectionality();
                switch (dir)
                {
                    case LEFT:
                    {
                        return DcasFunctionResult.CHANGE_LEFT;
                    }
                    case RIGHT:
                    {
                        return DcasFunctionResult.CHANGE_RIGHT;
                    }
                    case NONE:
                    default:
                        return DcasFunctionResult.NONE;
                }
            }
            return DcasFunctionResult.NONE;
        }
        catch (OperationalPlanException | ParameterException ex)
        {
            throw new OtsRuntimeException("Unabled to obtain information for PredicateLaneDrop", ex);
        }
    }

    /**
     * Returns most critical lane change information, or {@code null} when none within range.
     * @param context tactical context
     * @return most critical lane change information, or {@code null} when none within range
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private static LaneChangeInfo getLaneChangeInfo(final TacticalContextEgo context)
            throws OperationalPlanException, ParameterException
    {
        /*
         * Note: for now we rely on InfrastructurePerception, which is the perception category for the human model. Current
         * implementations return perfect information, but this may not hold true in the future. We add a defensive check.
         */
        InfrastructurePerception infra = context.getPerception().getPerceptionCategory(InfrastructurePerception.class);
        Throw.when(!(infra instanceof DirectInfrastructurePerception), IllegalStateException.class,
                "DcasFunctionInfrastructure only supports DirectInfrastructurePerception, but encountered %s",
                infra.getClass().getSimpleName());
        Length minDistPerLC = Length.POSITIVE_INFINITY;
        LaneChangeInfo mostCritical = null;
        Length maxRange = context.getParameters().getParameter(Dcas.X_NETWORK);
        for (LaneChangeInfo lcInfo : infra.getLegalLaneChangeInfo(RelativeLane.CURRENT))
        {
            if (lcInfo.remainingDistance().gt(maxRange))
            {
                return mostCritical;
            }
            Length distPerLc = lcInfo.remainingDistance().divide(lcInfo.numberOfLaneChanges());
            if (distPerLc.lt(minDistPerLC))
            {
                minDistPerLC = distPerLc;
                mostCritical = lcInfo;
            }
        }
        return mostCritical;
    }

}
