package org.opentrafficsim.dcas.tactical;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.draw.point.DirectedPoint2d;
import org.opentrafficsim.base.DistancedObject;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.logger.Logger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.TurnIndicatorStatus;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.perception.Historical;
import org.opentrafficsim.core.perception.HistoricalValue;
import org.opentrafficsim.dcas.tactical.Dcas.DcasState;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.gtu.operational.LaneOperationalPlanBuilder;
import org.opentrafficsim.road.gtu.operational.SimpleOperationalPlan;
import org.opentrafficsim.road.gtu.perception.LanePerception;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelTask;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;
import org.opentrafficsim.road.gtu.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.gtu.tactical.lmrs.AbstractIncentivesTacticalPlanner;
import org.opentrafficsim.road.gtu.tactical.util.DeadEndUtil;
import org.opentrafficsim.road.gtu.tactical.util.LaneChangeNotAllowedUtil;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.LmrsData;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.LmrsUtil;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.Tailgating;

/**
 * DcasTacticalPlanner.java.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasTacticalPlanner extends AbstractIncentivesTacticalPlanner
{

    /** Stimulus time for driver to change lane, increase acceleration, or reduce TOC TD. */
    public static final ParameterTypeDuration TAU_STIM = new ParameterTypeDuration("tauStim",
            "Stimulus time for driver to change lane, increase acceleration, or reduce TOC TD", Duration.ofSI(5.0),
            NumericConstraint.POSITIVE);

    /** Deviation object in case of no desired deviation. */
    private static final DistancedObject<Length> NO_DEVIATION = new DistancedObject<>(Length.ZERO, Length.ZERO);

    /** DCAS. */
    private final Dcas dcas;

    /** LMRS data. */
    private final LmrsData lmrsData;

    // Control flow

    /** Whether DCAS is requesting Transition Of Control. */
    private Historical<Boolean> toc;

    /** Whether the next step is DCAS or human behavior. */
    private boolean nextStepIsDcas = false;

    /** Time of next DCAS execution. */
    private Duration nextDcasTime;

    /** Time of next human model execution. */
    private Duration nextHumanTime;

    // Stimuli

    /** Time at which lane change stimulus started. */
    private Duration startLcStimulus;

    /** Lane change request. */
    private LateralDirectionality lcRequest = LateralDirectionality.NONE;

    /** Time at which acceleration stimulus started. */
    private Duration startAccelerationStimulus;

    /** Acceleration request (i.e. foot on pedal). */
    private Acceleration accelerationRequest;

    /** Dynamic time threshold for effective acceleration request trigger. */
    private Duration accelerationRequestThreshold;

    /**
     * Constructor.
     * @param carFollowingModel car-following model
     * @param gtu GTU
     * @param lanePerception perception
     * @param synchronization type of synchronization
     * @param cooperation type of cooperation
     * @param gapAcceptance gap-acceptance
     * @param tailgating tailgating
     */
    public DcasTacticalPlanner(final CarFollowingModel carFollowingModel, final LaneBasedGtu gtu,
            final LanePerception lanePerception, final Synchronization synchronization, final Cooperation cooperation,
            final GapAcceptance gapAcceptance, final Tailgating tailgating)
    {
        super(carFollowingModel, gtu, lanePerception);
        this.lmrsData = new LmrsData(synchronization, cooperation, gapAcceptance, tailgating);
        this.toc = new HistoricalValue<Boolean>(gtu.getSimulator().getReplication().getHistoryManager(gtu.getSimulator()), this,
                false);
        this.dcas = new Dcas((b) -> this.toc.set(b), () -> this.lcRequest);

        // add components that cannot be added through LmrsFactory
        lanePerception.addPerceptionCategory(new PerceptionCategoryToc(lanePerception));
        if (lanePerception.getMental().isPresent())
        {
            if (lanePerception.getMental().get() instanceof ChannelFuller channelFuller)
            {
                Set<ChannelTask> channelTaskToc = Set.of(new ChannelTaskToc());
                channelFuller.addTaskSupplier((lp) -> channelTaskToc);
            }
            else
            {
                throw new OtsRuntimeException("Using a mental model that is not supported by DcasTacticalPlanner ("
                        + lanePerception.getMental().get().getClass().getSimpleName() + ")");
            }
        }
    }

    @Override
    public OperationalPlan generateOperationalPlan(final Duration startTime, final DirectedPoint2d locationAtStartTime)
            throws GtuException, NetworkException, ParameterException
    {
        if (this.nextDcasTime == null)
        {
            this.nextDcasTime = Duration.POSITIVE_INFINITY;
            this.nextHumanTime = startTime;
        }

        TacticalContextEgo context = new TacticalContextEgo(getGtu());
        SimpleOperationalPlan simplePlan = controlFlow(context, startTime);

        // deal with dead-end situations and lane changes that are not allowed
        simplePlan = LaneChangeNotAllowedUtil.preventLaneChange(context, DeadEndUtil.dealWithDeadEnd(context, simplePlan));

        // set turn indicator
        context.getIntent(TurnIndicatorStatus.class).ifPresentOrElse((d) -> getGtu().setTurnIndicatorStatus(d.object()),
                () -> getGtu().setTurnIndicatorStatus(TurnIndicatorStatus.NONE));

        // create plan
        return LaneOperationalPlanBuilder.buildPlanFromSimplePlan(getGtu(), simplePlan,
                getGtu().getParameters().getParameter(ParameterTypes.LCDUR),
                context.getIntent(Length.class).orElse(NO_DEVIATION));
    }

    /**
     * Returns a simple plan as following from the control flow.
     * @param context tactical context
     * @param startTime start time of plan
     * @return simple plan
     * @throws OperationalPlanException when perception category is missing
     * @throws ParameterException when parameter is not available
     * @throws GtuException GTU exception
     * @throws NetworkException network exception
     */
    private SimpleOperationalPlan controlFlow(final TacticalContextEgo context, final Duration startTime)
            throws OperationalPlanException, ParameterException, GtuException, NetworkException
    {
        SimpleOperationalPlan simplePlan;
        if (this.nextStepIsDcas)
        {
            simplePlan = dcasModel(context, startTime);
        }
        else
        {
            if (this.dcas.isEnabled())
            {
                if (context.getPerception().getPerceptionCategory(PerceptionCategoryToc.class).getTransitionOfControlRequest())
                {
                    Logger.ots().trace("GTU " + getGtu().getId() + " disabled DCAS");
                    this.dcas.setEnabled(false);
                    this.nextDcasTime = Duration.POSITIVE_INFINITY;
                    simplePlan = humanModel(context, startTime);
                }
                else
                {
                    // if DCAS is on, the human model only functions as context for an LC request and acceleration overruling
                    humanModel(context, startTime);
                    simplePlan = new SimpleOperationalPlan(context.getAcceleration(), this.nextDcasTime.minus(startTime));
                }
            }
            else
            {
                if (canEnableDcas(context))
                {
                    Logger.ots().trace("GTU " + getGtu().getId() + " enabled DCAS");
                    this.dcas.setDesiredSpeed(context.getDesiredSpeed());
                    this.dcas.setEnabled(true);
                    simplePlan = dcasModel(context, startTime);
                }
                else
                {
                    simplePlan = humanModel(context, startTime);
                }
            }
        }
        this.nextStepIsDcas = this.dcas.isEnabled() && this.nextDcasTime.lt(this.nextHumanTime);
        // plan can't be super short, we get numerical errors when creating the path
        if (simplePlan.getDuration().si < 1e-3)
        {
            simplePlan = new SimpleOperationalPlan(simplePlan.getAcceleration(), Duration.ofSI(1e-3),
                    simplePlan.getLaneChangeDirection());
        }
        return simplePlan;
    }

    /**
     * Perform DCAS model.
     * @param context tactical context
     * @param startTime start time of plan
     * @return simple plan from DCAS
     * @throws OperationalPlanException when perception category is missing
     * @throws ParameterException when parameter is not available
     */
    private SimpleOperationalPlan dcasModel(final TacticalContextEgo context, final Duration startTime)
            throws OperationalPlanException, ParameterException
    {
        SimpleOperationalPlan simplePlan = this.dcas.getSimplePlan(context);
        this.nextDcasTime = startTime.plus(simplePlan.getDuration());

        // limit time of this plan
        if (this.nextDcasTime.gt(this.nextHumanTime))
        {
            simplePlan = new SimpleOperationalPlan(simplePlan.getAcceleration(), this.nextHumanTime.minus(startTime),
                    simplePlan.getLaneChangeDirection());
        }

        // cancel any user request upon a lane change (even though acceleration is continual foot pressure)
        if (!simplePlan.getLaneChangeDirection().isNone())
        {
            this.startLcStimulus = null;
            this.lcRequest = LateralDirectionality.NONE;
            this.startAccelerationStimulus = null;
            this.accelerationRequest = null;
        }

        // overrule with user acceleration
        if (this.accelerationRequest != null && this.accelerationRequest.gt(simplePlan.getAcceleration()))
        {
            simplePlan.setAcceleration(this.accelerationRequest);
        }
        return simplePlan;
    }

    /**
     * Executes the human model, which is the regular LMRS with whatever perception was setup.
     * @param context tactical context
     * @param startTime start time of plan
     * @return simple plan from DCAS
     * @throws ParameterException when parameter is missing
     * @throws GtuException GTU exception
     * @throws NetworkException when parameter is not available
     */
    private SimpleOperationalPlan humanModel(final TacticalContextEgo context, final Duration startTime)
            throws ParameterException, GtuException, NetworkException
    {
        this.dcas.setDesiredSpeed(context.getDesiredSpeed());
        SimpleOperationalPlan simplePlan = LmrsUtil.determinePlan(context, this.lmrsData, this);
        setLcRequest(context, startTime);
        setAccelerationRequest(context, startTime);

        this.nextHumanTime = startTime.plus(simplePlan.getDuration());
        if (this.nextHumanTime.gt(this.nextDcasTime) && this.nextDcasTime.gt(startTime))
        {
            simplePlan = new SimpleOperationalPlan(simplePlan.getAcceleration(), this.nextDcasTime.minus(startTime),
                    simplePlan.getLaneChangeDirection());
        }
        return simplePlan;
    }

    /**
     * Returns whether the driver want to enable DCAS.
     * @param context tactical context
     * @return whether the driver want to enable DCAS
     * @throws ParameterException when parameter is not available
     */
    private static boolean canEnableDcas(final TacticalContextEgo context) throws ParameterException
    {
        double dFree = context.getParameters().getParameter(LmrsParameters.DFREE);
        double dLeft = context.getParameters().getParameter(LmrsParameters.DLEFT);
        double dRight = context.getParameters().getParameter(LmrsParameters.DRIGHT);
        return dLeft < dFree && dRight < dFree && context.getAcceleration().ge0();
    }

    /**
     * Initiates, cancels or continues lane change request. The request is initiated or continued if lane change desire has been
     * above {@code DFREE} for over {@code REQUEST_DELAY}, or when lane change desire is over {@code DSYNC}. Otherwise it is
     * canceled or not initiated.
     * @param context tactical context
     * @param startTime start time of plan
     * @throws ParameterException when parameter is not available
     */
    private void setLcRequest(final TacticalContextEgo context, final Duration startTime) throws ParameterException
    {
        double dLeft = context.getParameters().getParameter(LmrsParameters.DLEFT);
        double dRight = context.getParameters().getParameter(LmrsParameters.DRIGHT);
        double d = Math.max(dLeft, dRight);
        double dFree = context.getParameters().getParameter(LmrsParameters.DFREE);

        inertialStimulus(startTime, () -> d >= dFree, this.startLcStimulus, (t) -> this.startLcStimulus = t);

        if (d >= context.getParameters().getParameter(LmrsParameters.DSYNC) || (this.startLcStimulus != null
                && startTime.minus(this.startLcStimulus).ge(context.getParameters().getParameter(TAU_STIM))))
        {
            this.lcRequest = dLeft >= dRight ? LateralDirectionality.LEFT : LateralDirectionality.RIGHT;
        }
        else
        {
            this.lcRequest = LateralDirectionality.NONE;
        }
    }

    /**
     * Initiates, cancels or continues acceleration request. The request is initiated or continued if speed has been below
     * {@code v0 - vGain} for over {@code REQUEST_DELAY}. Otherwise it is cancelled or not initiated.
     * @param context tactical context
     * @param startTime start time of plan
     * @throws ParameterException when parameter is not available
     */
    private void setAccelerationRequest(final TacticalContextEgo context, final Duration startTime) throws ParameterException
    {
        Speed vGain = context.getParameters().getParameter(LmrsParameters.VGAIN);
        Speed delta = context.getDesiredSpeed().minus(context.getSpeed());

        inertialStimulus(startTime, () -> delta.gt(vGain), this.startAccelerationStimulus,
                (t) -> this.startAccelerationStimulus = t);

        Duration tRequest = context.getParameters().getParameter(TAU_STIM);
        if (this.accelerationRequestThreshold == null)
        {
            this.accelerationRequestThreshold = tRequest;
        }
        if (this.startAccelerationStimulus != null
                && startTime.minus(this.startAccelerationStimulus).ge(this.accelerationRequestThreshold))
        {
            this.accelerationRequest = context.getCarFollowingAcceleration();
            this.accelerationRequestThreshold = Duration.ZERO;
        }
        /*
         * The time threshold for accelerating is not simply REQUEST_DELAY. This would cause a pronounced saw-tooth speed
         * profile as the driver accelerates up to v0-vGain, waits REQUEST_DELAY as the system slows down, and accelerates
         * again. Instead the used time threshold is set to 0s when acceleration starts. Consecutively its exponentially relaxed
         * to the normal value of REQUEST_DELAY, with a relaxation time of TAU (25s by default). This will mean that we have
         * "0 < threshold < REQUEST_DELAY" after speed has reached v0-vGain and the system slowly decelerates again. The driver
         * will then sooner than REQUEST_DELAY accelerate again. How soon depends on how recent the last acceleration started.
         */
        double ratio =
                context.getParameters().getParameter(DT).si / context.getParameters().getParameter(TAU_STIM).si;
        this.accelerationRequestThreshold =
                Duration.interpolate(this.accelerationRequestThreshold, tRequest, ratio <= 1.0 ? ratio : 1.0);
    }

    /**
     * Applies inertia to a stimulus.
     * @param startTime start time of plan
     * @param stimulus current stimulus provider
     * @param stored stored value
     * @param setter setter of given value as stored value
     */
    private void inertialStimulus(final Duration startTime, final Supplier<Boolean> stimulus, final Duration stored,
            final Consumer<Duration> setter)
    {
        if (stimulus.get())
        {
            if (stored == null)
            {
                setter.accept(startTime);
            }
        }
        else
        {
            setter.accept(null);
        }
    }

    /**
     * Returns whether there is a Transition Of Control request at the given time.
     * @param time time
     * @return whether there is a Transition Of Control request at the given time
     */
    public boolean getTransitionOfControlRequest(final Duration time)
    {
        return this.toc.get(time);
    }

    /**
     * Returns DCAS state.
     * @return DCAS state
     */
    public DcasState getState()
    {
        return this.dcas.getState();
    }

}
