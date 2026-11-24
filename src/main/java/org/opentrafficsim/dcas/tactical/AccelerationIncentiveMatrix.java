package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.Stateless;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.ControlType;
import org.opentrafficsim.dcas.tactical.MatrixPerceptionCategory.PerceivedMatrixSign;
import org.opentrafficsim.road.gtu.perception.PerceptionIterableSet;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;
import org.opentrafficsim.road.gtu.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.tactical.util.CarFollowingUtil;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.network.speed.SpeedLimit;
import org.opentrafficsim.road.network.speed.SpeedLimits;

/**
 * Acceleration response to matrix signs. The response depends on the control type of the matrix sign. If the matrix sign is
 * dynamically controlled, the response is based on the car-following model with a reduced maximum comfortable deceleration. The
 * reduction of this deceleration is inversely proportional to the ego-speed sensitivity. If the matrix sign is statically
 * controlled, the response is in terms of desired speed.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class AccelerationIncentiveMatrix implements AccelerationIncentive, Stateless<AccelerationIncentiveMatrix>
{

    /** Singleton instance. */
    public static final AccelerationIncentiveMatrix SINGLETON = new AccelerationIncentiveMatrix();

    @Override
    public AccelerationIncentiveMatrix get()
    {
        return SINGLETON;
    }

    @Override
    public Acceleration accelerate(final TacticalContextEgo context, final RelativeLane lane, final Length mergeDistance)
            throws ParameterException, GtuException
    {
        var signs = context.getPerception().getPerceptionCategory(MatrixPerception.class).getMatrixSigns(lane);
        var leaders = context.getPerception().getPerceptionCategory(NeighborsPerception.class).getLeaders(lane);
        double vGain = context.getParameters().getParameter(LmrsParameters.VGAIN).si;
        double b = context.getParameters().getParameter(ParameterTypes.B).si;
        double b0 = context.getParameters().getParameter(ParameterTypes.B0).si;
        Length x0 = context.getParameters().getParameter(ParameterTypes.LOOKAHEAD);
        double aMatrix = Double.MAX_VALUE;
        for (PerceivedMatrixSign sign : signs)
        {
            if (sign.matrixState().isSpeedLimit())
            {
                double a;
                if (ControlType.DYNAMIC.equals(sign.controlType()))
                {
                    // dynamically controlled matrix sign: reduce maximum comfortable deceleration based on ego speed
                    double f = Math.min(0.999, vGain / sign.matrixState().getSpeed().si); // not 1.0 as b0 < b needs to apply
                    double bEff = b - f * (b - b0);
                    context.getParameters().setParameterResettable(ParameterTypes.B, Acceleration.ofSI(bEff));
                    a = leaders.isEmpty() ? CarFollowingUtil.followSingleLeader(context, x0, sign.matrixState().getSpeed()).si
                            : context.getCarFollowingModel().followingAcceleration(context.getParameters(), context.getSpeed(),
                                    context.getSpeedLimits(), context.getMaximumSpeed(), leaders).si;
                    context.getParameters().resetParameter(ParameterTypes.B);
                }
                else
                {
                    // statically controlled matrix sign: apply custom speed limit
                    a = adjustToSpeed(context, sign.matrixState().getSpeed()).si;
                }
                aMatrix = Math.max(-b, a);
            }
        }
        return Acceleration.ofSI(aMatrix);
    }

    // TODO: move this to CarFollowingUtil
    /**
     * Adjusts the speed of the ego vehicle to the given speed. The car-following model is invoked with empty leaders and
     * dedicated speed limits.
     * @param context tactical context
     * @param speed speed
     * @return acceleration to adjust to the given speed
     * @throws ParameterException on missing parameter
     */
    private static Acceleration adjustToSpeed(final TacticalContextEgo context, final Speed speed) throws ParameterException
    {
        SpeedLimits speedLimits = new SpeedLimits(new SpeedLimit(speed, false), context.getSpeedLimits().gtuTypeSpeedLimit());
        PerceptionIterableSet<PerceivedObject> leaders = new PerceptionIterableSet<>();
        return context.getCarFollowingModel().followingAcceleration(context.getParameters(), context.getSpeed(), speedLimits,
                context.getMaximumSpeed(), leaders);
    }

}
