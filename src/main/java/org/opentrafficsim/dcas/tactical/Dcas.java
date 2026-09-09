package org.opentrafficsim.dcas.tactical;

import java.util.LinkedHashSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.exceptions.Throw;
import org.djutils.exceptions.Try;
import org.opentrafficsim.base.DistancedObject;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypeAcceleration;
import org.opentrafficsim.base.parameters.ParameterTypeBoolean;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.TurnIndicatorStatus;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.dcas.scenario.tools.Assumptions;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.gtu.operational.SimpleOperationalPlan;
import org.opentrafficsim.road.gtu.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.perception.PerceptionIterableSet;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.InfrastructurePerception;
import org.opentrafficsim.road.gtu.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.perception.object.PerceivedGtu;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject.Kinematics;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject.Kinematics.Overlap;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject.ObjectType;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObjectBase;
import org.opentrafficsim.road.gtu.perception.structure.LaneRecord;
import org.opentrafficsim.road.gtu.perception.structure.LaneStructure;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;
import org.opentrafficsim.road.gtu.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.gtu.tactical.following.DesiredHeadwayModel;
import org.opentrafficsim.road.gtu.tactical.following.DesiredSpeedModel;
import org.opentrafficsim.road.gtu.tactical.following.IdmPlus;
import org.opentrafficsim.road.network.Shoulder;

/**
 * DCAS (Driver Control Assistance System, ~SAE Level 2) functionality. This class can be used as a component within a tactical
 * planner in that it implements {@link DcasUserInterface}. Interaction between the tactical planner and this component operates
 * as follows.
 * <ul>
 * <li>This class is supplied a {@code Consumer<Boolean>} through which a Transition Of Control request signal is sent each time
 * this component runs. This signal is either {@code true} or {@code false}. For the tactical planner this can be as simple as
 * setting an internal property, e.g. {@code (b) -> this.toc = b}, which it uses whenever human behavior will run.</li>
 * <li>This class is supplied a {@code Supplier<LateralDirectionality>} through which the current state of a user-requested lane
 * change is supplied to this component upon its request. For the tactical planner this can be a simple forward of an internal
 * property, e.g. {@code () -> this.lcRequest}.</li>
 * <li>This class is supplied a {@code Supplier<Acceleration>} through which the current state of a user-requested throttle
 * acceleration is supplied to this component upon its request. For the tactical planner this can be a simple forward of an
 * internal property, e.g. {@code () -> this.throttleRequest}.</li>
 * <li>This class is supplied a {@code Supplier<Acceleration>} through which the current state of a user-requested brake
 * acceleration is supplied to this component upon its request. For the tactical planner this can be a simple forward of an
 * internal property, e.g. {@code () -> this.brakeRequest}.</li>
 * <li>The method {@link #getSimplePlan} can be invoked by the tactical planner to let the DCAS system run. The resulting plan
 * will have a duration equal to the system time. It is up to the tactical planner to, after that time, obtain a plan again.
 * That is, assuming the system was not disabled in the meantime.</li>
 * <li>The tactical planner can use {@link #isEnabled} and {@link #setEnabled} to control the enabled state of the system. Other
 * than checks, this has no effect on the internal mechanisms of the system. This information is for the tactical planner.</li>
 * <li>The tactical planner should use {@link #setUserSpeed} to set the speed at which DCAS will drive. The system has no
 * autonomous way to determine the speed to drive at and does not regard signs or other information. For internal reasons the
 * system may drive at a different speed.</li>
 * <li>Method {@link #getAcceleration} can be used by the tactical planner to continue a previous plan, that was interrupted for
 * reasons of the tactical planner that are not part of DCAS functionality itself. For example, for some intermediate human
 * evaluation step that results in no action of the human driver.</li>
 * <li>Remaining public methods are implementations of methods in {@link DcasSystemInterface} and are for internal DCAS
 * functions. Functions added to the system can use these methods.</li>
 * </ul>
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class Dcas implements DcasSystemInterface, DcasUserInterface
{

    /** Length over which DCAS is network and signs aware. */
    public static final ParameterTypeLength X_NETWORK = new ParameterTypeLength("xNetwork",
            "Length over which DCAS is network aware", Assumptions.get().dcas().xNetwork(), NumericConstraint.POSITIVE);

    /** Maximum deceleration DCAS may perform. */
    public static final ParameterTypeAcceleration MAX_B_DCAS = new ParameterTypeAcceleration("maxBDcas",
            "Maximum deceleration DCAS may perform", Assumptions.get().dcas().cf().maxBDcas(), NumericConstraint.POSITIVE);

    /** Maximum deceleration DCAS may perform. */
    public static final ParameterTypeDuration MIN_TTC_DCAS =
            new ParameterTypeDuration("minTtcDcas", "Minimum Time To Collision for lane change",
                    Assumptions.get().dcas().lc().minTtcDcas(), NumericConstraint.POSITIVE);

    /** Minimum time headway for lane change. */
    public static final ParameterTypeDuration MIN_T_DCAS = new ParameterTypeDuration("minTDcas",
            "Minimum time headway for lane change", Assumptions.get().dcas().lc().minTDcas(), NumericConstraint.POSITIVE);

    /** Time step of DCAS system. */
    public static final ParameterTypeDuration DT_DCAS = new ParameterTypeDuration("dtDcas", "Time step of DCAS system",
            Assumptions.get().dcas().dtDcas(), NumericConstraint.POSITIVE);

    /** System is lane change able. */
    public static final ParameterTypeBoolean LC_DCAS =
            new ParameterTypeBoolean("lcDcas", "System is lane change able", Assumptions.get().dcas().lc().lcDcas());

    /** System is shoulder Minimum Risk Maneuver able. */
    public static final ParameterTypeBoolean SHOULDER_DCAS = new ParameterTypeBoolean("shoulderDcas",
            "System is shoulder Minimum Risk Maneuver able", Assumptions.get().dcas().lc().shoulderDcas());

    /** System is shoulder Minimum Risk Maneuver able. */
    public static final ParameterTypeDuration TOC_ESCALATE =
            new ParameterTypeDuration("tocEscDcas", "Time after which a Transition Of Control request is escalated",
                    Assumptions.get().dcas().tocEscDcas(), NumericConstraint.POSITIVE);

    /** Settings for DCAS. */
    private final Parameters settings;

    /** Enabled state of DCAS. */
    private boolean enabled = false;

    /** Car-following model. */
    private final CarFollowingModel carFollowingModel;

    /** Consumer that will be activated when DCAS requests transition of control. */
    private final Consumer<TocRequestLevel> transitionOfControl;

    /** User throttle request. */
    private final Supplier<Acceleration> userThrottleRequest;

    /** User brake request. */
    private final Supplier<Acceleration> userBrakeRequest;

    /** DCAS functions. */
    private final Set<DcasFunction> functions = new LinkedHashSet<>();

    /** User speed. */
    private Speed userSpeed;

    /** Speed to apply when lower than user speed, as set by a DCAS function. */
    private Speed loweredSystemSpeed;

    /** Acceleration as determined by the DCAS functions. */
    private Acceleration acceleration;

    /** Last operation was at Transition Of Control priority. */
    private TocRequestLevel transitionOfControlLevel = TocRequestLevel.OFF;

    /** Time at which tTransition Of Control (TOC) request started. */
    private Duration transitionOfControlStart;

    /** Last operation was at Minimal Risk Maneuver priority. */
    private boolean minimalRiskManeuverState = false;

    /** Last operation was at lane change priority, but the gap was not accepted. */
    private boolean synchronizing = false;

    /**
     * Constructor.
     * @param settings DCAS settings
     * @param transitionOfControl consumer that will be activated when DCAS requests transition of control
     * @param userLcRequest supplier of the latest user lane change request
     * @param userThrottleRequest supplier of the latest user throttle request, which may be {@code null}
     * @param userBrakeRequest supplier of the latest user brake request, which may be {@code null}
     */
    public Dcas(final Parameters settings, final Consumer<TocRequestLevel> transitionOfControl,
            final Supplier<LateralDirectionality> userLcRequest, final Supplier<Acceleration> userThrottleRequest,
            final Supplier<Acceleration> userBrakeRequest)
    {
        this.settings = settings;
        // The desired headway and speed model for DCAS ignore the parameters from the GTU, and use local settings instead
        DesiredHeadwayModel sModel = (params, v) -> Length.ofSI(
                this.settings.getParameter(ParameterTypes.S0).si + this.settings.getParameter(ParameterTypes.T).si * v.si);
        DesiredSpeedModel vModel = (params, vLims, vMax) -> Speed.min(vMax,
                this.loweredSystemSpeed == null ? this.userSpeed : Speed.min(this.loweredSystemSpeed, this.userSpeed));
        this.carFollowingModel = new IdmPlus(sModel, vModel);
        this.transitionOfControl = transitionOfControl;
        this.userThrottleRequest = userThrottleRequest;
        this.userBrakeRequest = userBrakeRequest;

        this.functions.add(new DcasFunctionInfrastructure());
        this.functions.add(new DcasFunctionUserLcRequest(userLcRequest)); // might translate request in DcasFunctionResult
        this.functions.add(new DcasFunctionCarFollowing());
        this.functions.add(new DcasFunctionCooperate());
    }

    @Override
    public void setEnabled(final boolean enabled)
    {
        this.enabled = enabled;
    }

    @Override
    public boolean isEnabled()
    {
        return this.enabled;
    }

    @Override
    public void setUserSpeed(final Speed userSpeed)
    {
        this.userSpeed = userSpeed;
    }

    @Override
    public void setLoweredSystemSpeed(final Speed loweredSystemSpeed)
    {
        this.loweredSystemSpeed =
                this.loweredSystemSpeed == null ? loweredSystemSpeed : Speed.min(this.loweredSystemSpeed, loweredSystemSpeed);
    }

    @Override
    public void setSystemAcceleration(final Acceleration systemAcceleration)
    {
        this.acceleration =
                this.acceleration == null ? systemAcceleration : Acceleration.min(this.acceleration, systemAcceleration);
    }

    @Override
    public Acceleration getAcceleration()
    {
        return this.acceleration;
    }

    @Override
    public <T> T getSetting(final ParameterType<T> setting)
    {
        return Try.assign(() -> this.settings.getParameter(setting), OtsRuntimeException.class, "Missing setting %s",
                setting.getId());
    }

    /**
     * {@inheritDoc} This follows one leader based on exact information and using a local IDM+ with local parameters.
     */
    @Override
    public Acceleration getCarFollowingAcceleration(final TacticalContextEgo context, final RelativeLane lane)
    {
        try
        {
            PerceptionCollectable<PerceivedGtu, LaneBasedGtu> leaders =
                    context.getPerception().getPerceptionCategory(NeighborsPerception.class).getLeaders(lane);
            PerceptionIterableSet<PerceivedObject> leader;
            if (leaders.isEmpty())
            {
                leader = new PerceptionIterableSet<>();
            }
            else
            {
                // take exact leader information from underlying object
                DistancedObject<LaneBasedGtu> rawLeader = leaders.underlyingWithDistance().next();
                leader = new PerceptionIterableSet<>(
                        new PerceivedObjectBase(rawLeader.object().getId(), ObjectType.GTU, Length.ONE, new Kinematics.Record(
                                rawLeader.distance(), rawLeader.object().getSpeed(), Acceleration.ZERO, true, Overlap.AHEAD)));
            }
            return this.carFollowingModel.followingAcceleration(this.settings, context.getSpeed(), context.getSpeedLimits(),
                    context.getMaximumSpeed(), leader);
        }
        catch (OperationalPlanException | ParameterException ex)
        {
            throw new OtsRuntimeException("Unable to determine car-following acceleration of DCAS system.", ex);
        }
    }

    @Override
    public SimpleOperationalPlan getSimplePlan(final TacticalContextEgo context)
            throws OperationalPlanException, ParameterException
    {
        Throw.when(!this.enabled, IllegalStateException.class,
                "DCAS is requested to return a plan, but the system is set to disabled.");

        this.loweredSystemSpeed = null;
        this.acceleration = null;
        DcasFunctionResult dcasFunctionResult = applyFunctions(context);

        if (dcasFunctionResult.ordinal() == DcasFunctionResult.TRANSITION_OF_CONTROL.ordinal())
        {
            if (this.transitionOfControlStart == null)
            {
                this.transitionOfControlStart = context.getTime();
            }
            this.transitionOfControlLevel =
                    context.getTime().minus(this.transitionOfControlStart).gt(this.settings.getParameter(TOC_ESCALATE))
                            ? TocRequestLevel.HIGH : TocRequestLevel.LOW;
        }
        else
        {
            this.transitionOfControlLevel = TocRequestLevel.OFF;
            this.transitionOfControlStart = null;
        }
        this.minimalRiskManeuverState = dcasFunctionResult.ordinal() > DcasFunctionResult.TRANSITION_OF_CONTROL.ordinal();
        this.transitionOfControl.accept(this.transitionOfControlLevel);

        Acceleration throttle = this.userThrottleRequest.get();
        if (throttle != null)
        {
            this.acceleration = Acceleration.max(this.acceleration, throttle);
        }
        Acceleration brake = this.userBrakeRequest.get();
        if (brake != null)
        {
            this.acceleration = Acceleration.min(this.acceleration, brake);
        }

        Optional<Acceleration> stop = considerStop(context, dcasFunctionResult);
        this.acceleration = stop.isPresent() ? Acceleration.min(stop.get(), this.acceleration) : this.acceleration;
        LateralDirectionality lc = determineLaneChange(context, dcasFunctionResult);

        return new SimpleOperationalPlan(this.acceleration, this.settings.getParameter(DT_DCAS), lc);
    }

    @Override
    public DcasState getState()
    {
        if (!this.enabled)
        {
            return DcasState.OFF;
        }
        if (this.minimalRiskManeuverState)
        {
            return DcasState.MRM;
        }
        if (!this.transitionOfControlLevel.equals(TocRequestLevel.OFF))
        {
            return DcasState.TOC;
        }
        if (this.synchronizing)
        {
            return DcasState.SYNC;
        }
        return DcasState.ON;
    }

    /**
     * Returns acceleration to stop, if applicable.
     * @param context tactical context
     * @param dcasFunctionResult DCAS function result
     * @return acceleration to stop, if applicable
     * @throws ParameterException when a parameter is not available
     */
    private Optional<Acceleration> considerStop(final TacticalContextEgo context, final DcasFunctionResult dcasFunctionResult)
            throws ParameterException
    {
        if (dcasFunctionResult.ordinal() >= DcasFunctionResult.STOP.ordinal())
        {
            context.addIntent(TurnIndicatorStatus.HAZARD, Length.ZERO);
            // Slow down when DcasFunctionResult.STOP, when DcasFunctionResult.SHOULDER but system is not shoulder-able,
            // or when on shoulder
            if (dcasFunctionResult.equals(DcasFunctionResult.STOP)
                    || (dcasFunctionResult.equals(DcasFunctionResult.SHOULDER) && !this.settings.getParameter(SHOULDER_DCAS))
                    || context.getPerception().getLaneStructure().getRootRecord(RelativeLane.CURRENT)
                            .getLane() instanceof Shoulder)
            {
                return Optional.of(this.settings.getParameter(ParameterTypes.B0).neg());
            }
        }
        return Optional.empty();
    }

    /**
     * Returns the direction to change lane in.
     * @param context tactical context
     * @param dcasFunctionResult DCAS function result
     * @return direction to change lane in, may be {@code NONE}
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private LateralDirectionality determineLaneChange(final TacticalContextEgo context,
            final DcasFunctionResult dcasFunctionResult) throws OperationalPlanException, ParameterException
    {
        this.synchronizing = false;
        boolean lcAble = this.settings.getParameter(LC_DCAS);
        LateralDirectionality dir = switch (dcasFunctionResult)
        {
            case CHANGE_LEFT -> LateralDirectionality.LEFT;
            case CHANGE_RIGHT -> LateralDirectionality.RIGHT;
            default -> null;
        };
        if (lcAble && dir != null)
        {
            if (acceptGap(context, dir, true))
            {
                return dir;
            }
            // synchronize
            this.synchronizing = true;
            setSystemAcceleration(Acceleration.max(this.settings.getParameter(ParameterTypes.B).neg(),
                    getCarFollowingAcceleration(context, dir.isLeft() ? RelativeLane.LEFT : RelativeLane.RIGHT)));
        }
        else if (this.settings.getParameter(SHOULDER_DCAS)
                && dcasFunctionResult.ordinal() >= DcasFunctionResult.SHOULDER.ordinal())
        {
            return determineShoulderLaneChange(context);
        }
        return LateralDirectionality.NONE;
    }

    /**
     * Returns the direction to change lane in order to reach a shoulder.
     * @param context tactical context
     * @return direction to change lane in order to reach a shoulder, may be {@code NONE}
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private LateralDirectionality determineShoulderLaneChange(final TacticalContextEgo context)
            throws OperationalPlanException, ParameterException
    {
        LaneStructure structure = context.getPerception().getLaneStructure();
        LaneRecord rightRecord = structure.getRootRecord(RelativeLane.RIGHT);
        boolean rightInfra = rightRecord != null;
        boolean rightIsShoulder = rightInfra && rightRecord.getLane() instanceof Shoulder;
        boolean acceptRightGap = rightInfra ? acceptGapVehicles(context, LateralDirectionality.RIGHT) : false;
        if (rightIsShoulder && acceptRightGap)
        {
            // 1) if adjacent to right-hand shoulder and gap ok, change right
            return LateralDirectionality.RIGHT;
        }
        LaneRecord leftRecord = structure.getRootRecord(RelativeLane.LEFT);
        boolean leftIsShoulder = leftRecord != null && leftRecord.getLane() instanceof Shoulder;
        if (leftIsShoulder)
        {
            // 2) if adjacent to left-hand shoulder and gap ok, change left
            boolean acceptLeftGap = acceptGapVehicles(context, LateralDirectionality.LEFT);
            if (acceptLeftGap)
            {
                return LateralDirectionality.LEFT;
            }
            // 3) if adjacent to left-hand shoulder, and gap not ok, do not move to right-hand normal lane, postpone
            // or: one lane between two shoulders that both have unacceptable gaps, also do not change lane
            return LateralDirectionality.NONE;
        }
        else if (rightInfra && acceptRightGap)
        {
            // 4) if there is a right lane and gap ok, change right
            return LateralDirectionality.RIGHT;
        }
        // 5) system currently sees no option
        return LateralDirectionality.NONE;
    }

    /**
     * Accept gap for changing lane.
     * @param context tactical context
     * @param direction potential lane change direction
     * @param legal whether to regard legal rules, or physical infrastructure
     * @return whether the gap is accepted
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private boolean acceptGap(final TacticalContextEgo context, final LateralDirectionality direction, final boolean legal)
            throws OperationalPlanException, ParameterException
    {
        InfrastructurePerception infra = context.getPerception().getPerceptionCategory(InfrastructurePerception.class);
        boolean infraAllows = legal ? infra.getLegalLaneChangePossibility(RelativeLane.CURRENT, direction).gt0()
                : infra.getPhysicalLaneChangePossibility(RelativeLane.CURRENT, direction).gt0();
        if (!infraAllows)
        {
            return false;
        }
        return acceptGapVehicles(context, direction);
    }

    /**
     * Check whether the system accepts the gaps relative to vehicles in the adjacent lane.
     * @param context tactical context
     * @param direction potential lane change direction
     * @return whether the gap is accepted
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private boolean acceptGapVehicles(final TacticalContextEgo context, final LateralDirectionality direction)
            throws OperationalPlanException, ParameterException
    {
        RelativeLane lane = new RelativeLane(direction, 1);
        NeighborsPerception neighbors = context.getPerception().getPerceptionCategory(NeighborsPerception.class);
        if (neighbors.isGtuAlongside(direction))
        {
            return false;
        }
        PerceptionCollectable<PerceivedGtu, LaneBasedGtu> leaders = neighbors.getLeaders(lane);
        boolean leaderAllows = acceptGapVehicle(context, leaders, (vEgo, vAdj) -> vEgo, (vEgo, vAdj) -> vEgo.minus(vAdj));
        if (!leaderAllows)
        {
            return false;
        }
        PerceptionCollectable<PerceivedGtu, LaneBasedGtu> followers =
                context.getPerception().getPerceptionCategory(NeighborsPerception.class).getFollowers(lane);
        return acceptGapVehicle(context, followers, (vEgo, vAdj) -> vAdj, (vEgo, vAdj) -> vAdj.minus(vEgo));
    }

    /**
     * Check whether the system accepts the gap relative to a single vehicle in the adjacent lane.
     * @param context tactical context
     * @param adjacent leaders or followers
     * @param speed returns the speed of the upstream vehicle, being given both vehicle speed
     * @param dv returns the speed difference (upstream - downstream), being given both vehicle speed
     * @return whether the speed difference and distance to the first adjacent vehicle can be accepted for changing lane
     * @throws ParameterException when a parameter is not available
     */
    private boolean acceptGapVehicle(final TacticalContextEgo context,
            final PerceptionCollectable<PerceivedGtu, LaneBasedGtu> adjacent, final BiFunction<Speed, Speed, Speed> speed,
            final BiFunction<Speed, Speed, Speed> dv) throws ParameterException
    {
        if (!adjacent.isEmpty())
        {
            DistancedObject<LaneBasedGtu> raw = adjacent.underlyingWithDistance().next();
            Speed deltaV = dv.apply(context.getSpeed(), raw.object().getSpeed());
            return (deltaV.lt0() || raw.distance().divide(deltaV).ge(this.settings.getParameter(MIN_TTC_DCAS))) && raw
                    .distance()
                    .gt(this.settings.getParameter(MIN_T_DCAS).times(speed.apply(context.getSpeed(), raw.object().getSpeed())));
        }
        return true;
    }

    /**
     * Applies the DCAS functions and returns the result of highest priority. If any function is in a state of requesting
     * transition of control, a request is performed.
     * @param context tactical context
     * @return function result of highest priority
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    private DcasFunctionResult applyFunctions(final TacticalContextEgo context)
            throws OperationalPlanException, ParameterException
    {
        DcasFunctionResult result = DcasFunctionResult.NONE;
        for (DcasFunction function : this.functions)
        {
            DcasFunctionResult functionResult = function.apply(context, this);
            if (functionResult.ordinal() > result.ordinal())
            {
                result = functionResult;
            }
        }
        return result;
    }

    /**
     * Transition Of Control (TOC) request level.
     */
    public enum TocRequestLevel
    {

        /** Off. */
        OFF,

        /** Low. */
        LOW,

        /** High. */
        HIGH;

    }

}
