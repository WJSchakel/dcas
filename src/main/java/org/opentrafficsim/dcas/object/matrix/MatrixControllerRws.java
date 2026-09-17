package org.opentrafficsim.dcas.object.matrix;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.geometry.FractionalProjectionHelper.FractionalFallback;
import org.opentrafficsim.base.logger.Logger;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.object.DetectorType;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.MatrixState;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.network.CrossSectionLink;
import org.opentrafficsim.road.network.Lane;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.object.LaneBasedObject;
import org.opentrafficsim.road.network.object.detector.LaneDetector;

import nl.tudelft.simulation.dsol.formalisms.eventscheduling.SimEventInterface;

/**
 * Standard controller for matrix signs that mimics the Rijkswaterstaat (RWS) algorithm in Dutch highways. This controller sets
 * itself up by finding all {@link MatrixSign}, grouping them logically by position and possibly by stream at weaving sections,
 * coupling to upstream groups, and setting up detectors.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class MatrixControllerRws
{

    /** Distance within which two matrix signs will be assigned to the same group (e.g. gantry). */
    private static final Length GROUPING_LENGTH = Length.ofSI(5.0);

    /** Maximum link distance over which an upstream group is found and set as upstream group of a group. */
    private static final Length MAX_UPSTREAM_LENGTH = Length.ofSI(1000.0);

    /** Stored group per matrix sign. */
    private Map<MatrixSign, Group> groupOfSign = new LinkedHashMap<>();

    /** States and their priorities for each matrix sign for which some non-blank state is set. */
    private Map<MatrixSign, TreeMap<Priority, MatrixState>> states = new LinkedHashMap<>();

    /**
     * Constructor. This will find all matrix signs on the network and group them, couple the groups for automated signal
     * states, and create detectors and a controller to trigger the automated control.
     * @param network network
     * @param detectorType detector type
     */
    public MatrixControllerRws(final RoadNetwork network, final DetectorType detectorType)
    {
        findGroups(network);
        setupDetectors(detectorType);
    }

    /**
     * Finds and couples the groups.
     * @param network network
     */
    private void findGroups(final RoadNetwork network)
    {
        // Step 1: find all matrix signs and their position on a link
        Map<Link, TreeMap<Length, Set<MatrixSign>>> allMatrices = new LinkedHashMap<>();
        for (Link link : network.getLinkMap().values())
        {
            if (link instanceof CrossSectionLink cLink)
            {
                for (Lane lane : cLink.getLanesAndShoulders())
                {
                    for (LaneBasedObject object : lane.getLaneBasedObjects())
                    {
                        if (object instanceof MatrixSign matrix)
                        {
                            double f = link.getDesignLine().projectFractionalAt(link.getStartNode().getHeading(),
                                    link.getEndNode().getHeading(), object.getLocation().x, object.getLocation().y,
                                    FractionalFallback.ENDPOINT);
                            Length positionOnLink = link.getLength().times(f);
                            allMatrices.computeIfAbsent(link, (l) -> new TreeMap<>())
                                    .computeIfAbsent(positionOnLink, (p) -> new LinkedHashSet<>()).add(matrix);
                        }
                    }
                }
            }
        }

        // Step 2: group all matrix signs within a certain distance of one another on the link, but split per block line
        // -> sets lateral connection
        Map<Link, TreeMap<Length, Set<Group>>> allGroups = new LinkedHashMap<>();
        for (Entry<Link, TreeMap<Length, Set<MatrixSign>>> linkEntry : allMatrices.entrySet())
        {
            TreeMap<Length, Set<MatrixSign>> objectMap = linkEntry.getValue();
            Set<MatrixSign> matrixSigns = new LinkedHashSet<>();
            Length prevPos = null;
            for (Entry<Length, Set<MatrixSign>> objectEntry : objectMap.entrySet())
            {
                if (prevPos != null && Math.abs(prevPos.si - objectEntry.getKey().si) > GROUPING_LENGTH.si)
                {
                    makeGroupsInCrossSection(linkEntry.getKey(), matrixSigns, allGroups);
                    matrixSigns = new LinkedHashSet<>();
                }
                matrixSigns.addAll(objectEntry.getValue());
                prevPos = objectEntry.getKey();
            }
            if (!matrixSigns.isEmpty())
            {
                makeGroupsInCrossSection(linkEntry.getKey(), matrixSigns, allGroups);
            }
        }

        // Step 3: search upstream group in upstream link
        // -> sets upstream connection
        for (Entry<Link, TreeMap<Length, Set<Group>>> linkEntry : allGroups.entrySet())
        {
            for (Entry<Length, Set<Group>> positionEntry : linkEntry.getValue().entrySet())
            {
                Length position = positionEntry.getKey();
                for (Group group : positionEntry.getValue())
                {
                    setUpstreamGroups(group, linkEntry.getKey(), position, allGroups, Length.ZERO);
                }
            }
        }
    }

    /**
     * Takes matrix signs from a cross-section and creates one or more groups out of them.
     * @param link link
     * @param matrixSigns set of matrix signs from the same cross-section
     * @param allGroups map to write groups in to
     */
    private void makeGroupsInCrossSection(final Link link, final Set<MatrixSign> matrixSigns,
            final Map<Link, TreeMap<Length, Set<Group>>> allGroups)
    {
        // TODO lateral groups
        Group group = new Group(matrixSigns);
        matrixSigns.forEach((m) -> this.groupOfSign.put(m, group));
        Length position = matrixSigns.iterator().next().getLongitudinalPosition(); // any position for the cross-seciton will do
        allGroups.computeIfAbsent(link, (l) -> new TreeMap<>()).put(position, Set.of(group));
    }

    /**
     * Recursively searches upstream links to find upstream groups to respond to the given initial group.
     * @param initialGroup initial group to search upstream of
     * @param link current link to search on
     * @param position current position to search upstream of (initially position of initial group, link end otherwise)
     * @param allGroups all groups sorted per link
     * @param cumulativeDistance cumulative search distance, initially zero
     */
    private void setUpstreamGroups(final Group initialGroup, final Link link, final Length position,
            final Map<Link, TreeMap<Length, Set<Group>>> allGroups, final Length cumulativeDistance)
    {
        if (allGroups.containsKey(link))
        {
            Entry<Length, Set<Group>> upstream = allGroups.get(link).lowerEntry(position);
            if (upstream != null)
            {
                if (cumulativeDistance.si + (position.si - upstream.getKey().si) < MAX_UPSTREAM_LENGTH.si)
                {
                    Logger.ots().trace("Matrix group with {} upstream of group with {}.",
                            upstream.getValue().iterator().next().matrixSigns.iterator().next().getFullId(),
                            initialGroup.matrixSigns.iterator().next().getFullId());
                    upstream.getValue().forEach((u) -> initialGroup.upstream.add(u));
                }
                return;
            }
        }
        Length nextCumul = cumulativeDistance.plus(position);
        if (nextCumul.si < MAX_UPSTREAM_LENGTH.si)
        {
            for (Link startLink : link.getStartNode().getLinks())
            {
                if (startLink.getEndNode().equals(link.getStartNode()))
                {
                    setUpstreamGroups(initialGroup, startLink, startLink.getLength(), allGroups, nextCumul);
                }
            }
        }
    }

    /**
     * Sets up detectors and a controller that responds to them.
     * @param detectorType detector type
     */
    private void setupDetectors(final DetectorType detectorType)
    {
        for (MatrixSign matrix : this.groupOfSign.keySet())
        {
            try
            {
                new MatrixDetector(matrix, detectorType);
            }
            catch (NetworkException ex)
            {
                throw new OtsRuntimeException("Could not create a detector for a matrix sign.", ex);
            }
        }
    }

    /**
     * Allows a traffic management center to overrule the state of a matrix sign. Internal mechanisms will operate as usual, but
     * the displayed state will be the given one.
     * @param matrix matrix sign
     * @param state matrix state
     */
    public void setOverrulState(final MatrixSign matrix, final MatrixState state)
    {
        addState(matrix, state, Priority.OVERRULE);
    }

    /**
     * Remove overrule matrix state.
     * @param matrix matrix sign to remove the overruled state for
     */
    public void removeOverrulState(final MatrixSign matrix)
    {
        removeState(matrix, Priority.OVERRULE);
    }

    /**
     * Add a state to a matrix sign with priority. The state of the actual sign will be updated in accordance.
     * @param matrix matrix sign
     * @param state matrix state
     * @param priority state priority
     */
    private void addState(final MatrixSign matrix, final MatrixState state, final Priority priority)
    {
        this.states.computeIfAbsent(matrix, (m) -> new TreeMap<>()).put(priority, state);
        setHighestPriorityState(matrix);
    }

    /**
     * Remove state of given priority for the given matrix sign. The state of the actual sign will be updated in accordance.
     * @param matrix matrix sign
     * @param priority state priority
     */
    private void removeState(final MatrixSign matrix, final Priority priority)
    {
        TreeMap<Priority, MatrixState> mStates = this.states.get(matrix);
        if (mStates != null)
        {
            mStates.remove(priority);
        }
        setHighestPriorityState(matrix);
    }

    /**
     * Set the highest priority state on the given matrix sign.
     * @param matrix matrix sign
     */
    private void setHighestPriorityState(final MatrixSign matrix)
    {
        TreeMap<Priority, MatrixState> mStates = this.states.get(matrix);
        matrix.setState(mStates != null && !mStates.isEmpty() ? mStates.lastEntry().getValue() : MatrixState.BLANK);
    }

    /**
     * Helper class that represents a single group of matrix signs. Typically one gantry, but could be only partial if there are
     * multiple streams at the cross-section (for example at a weaving section).
     */
    private final class Group
    {

        /** Set of matrix signs in the group. */
        private final Set<MatrixSign> matrixSigns;

        /** Triggered signs. */
        private final Set<MatrixSign> triggered = new LinkedHashSet<>();

        /** Left group. */
        private Group left;

        /** Right group. */
        private Group right;

        /** Upstream groups (can be multiple if there is a merge upstream). */
        private Set<Group> upstream = new LinkedHashSet<>();

        /**
         * Constructor.
         * @param matrixSigns matrix signs
         */
        private Group(final Set<MatrixSign> matrixSigns)
        {
            this.matrixSigns = matrixSigns;
        }

        /**
         * Sets the group on, which is the starting point of setting it to 50km/h and then setting upstream groups.
         * @param matrix matrix sign of trigger
         */
        private void setOn(final MatrixSign matrix)
        {
            if (this.triggered.isEmpty())
            {
                Logger.ots().trace("Turning ON gantry for {}", matrix.getFullId());
                setSelf(Action.ADD);
            }
            this.triggered.add(matrix);
        }

        /**
         * Sets the group off, which undoes the upstream cascade of setting it on.
         * @param matrix matrix sign of trigger, the group remains on for as long as at least one matrix triggers the group
         */
        private void setOff(final MatrixSign matrix)
        {
            this.triggered.remove(matrix);
            if (this.triggered.isEmpty())
            {
                Logger.ots().trace("Turning OFF gantry for {}", matrix.getFullId());
                setSelf(Action.REMOVE);
            }
        }

        /**
         * Starts the upstream cascade to either set or remove matrix states.
         * @param action add or remove action
         */
        private void setSelf(final Action action)
        {
            this.matrixSigns.forEach((m) -> performAction(m, action, MatrixState.V50, Priority.SELF));
            this.upstream.forEach((g) -> g.setUpstream50(action));
            setLateral(action, MatrixState.V70, Priority.LATERAL_70);
        }

        /**
         * Sets the state of this group as the first upstream group of a group that was turned on.
         * @param action add or remove action
         */
        private void setUpstream50(final Action action)
        {
            this.matrixSigns.forEach((m) -> performAction(m, action, MatrixState.V50, Priority.UPSTREAM_50));
            setLateral(action, MatrixState.V70, Priority.LATERAL_70);
            this.upstream.forEach((g) -> g.setUpstream70(action));
        }

        /**
         * Sets the state of this group as the second upstream group of a group that was turned on.
         * @param action add or remove action
         */
        private void setUpstream70(final Action action)
        {
            this.matrixSigns.forEach((m) -> performAction(m, action, MatrixState.V70, Priority.UPSTREAM_70));
            setLateral(action, MatrixState.V90, Priority.LATERAL_90);
        }

        /**
         * Sets the state of this group as a lateral group to another group for which a state is set.
         * @param action add or remove action
         * @param state state to set
         * @param priority priority to add the state with, or to remove the state of
         */
        private void setLateral(final Action action, final MatrixState state, final Priority priority)
        {
            if (this.left != null)
            {
                this.left.matrixSigns.forEach((m) -> performAction(m, action, state, priority));
            }
            if (this.right != null)
            {
                this.right.matrixSigns.forEach((m) -> performAction(m, action, state, priority));
            }
        }

        /**
         * Perform a add or remove action.
         * @param matrix matrix
         * @param action add or remove action
         * @param state state to add
         * @param priority priority to add the state with, or to remove the state of
         */
        @SuppressWarnings("LeftCurly")
        private void performAction(final MatrixSign matrix, final Action action, final MatrixState state,
                final Priority priority)
        {
            switch (action)
            {
                case ADD -> addState(matrix, state, priority);
                case REMOVE -> removeState(matrix, priority);
                default -> {
                }
            }
        }

        /**
         * Add or remove action in upstream cascade.
         */
        private enum Action
        {

            /** Add. */
            ADD,

            /** Remove. */
            REMOVE;

        }
    }

    /**
     * Priority of a matrix state. The highest priority determines the state.
     */
    private enum Priority
    {

        /** 90km/h as an adjacent group was set to 70km/h. */
        LATERAL_90,

        /** 70km/h as an adjacent group was set to 50km/h. */
        LATERAL_70,

        /** 70km/h as the second upstream group of a triggered group. */
        UPSTREAM_70,

        /** 50km/h as the first upstream group of a triggered group. */
        UPSTREAM_50,

        /** 50km/h as this group was triggered. */
        SELF,

        /** Overruled from traffic management center. */
        OVERRULE;

    }

    /**
     * Detector that maintains automated signaling thresholds.
     */
    private final class MatrixDetector extends LaneDetector
    {

        /** Minimum number of vehicles for on trigger. */
        private static final int NUMBER = 3;

        /** Time period for hysteresis. */
        private static final Duration TIME = Duration.ofSI(30.0);

        /** Speed below which to turn on. */
        private static final Speed ON_SPEED = new Speed(35.0, SpeedUnit.KM_PER_HOUR);

        /** Speed above which to turn off. */
        private static final Speed OFF_SPEED = new Speed(55.0, SpeedUnit.KM_PER_HOUR);

        /** Matrix sign. */
        private final MatrixSign matrix;

        /** Time when trigger turned on. */
        private Duration on;

        /** Last scheduled check event. */
        private SimEventInterface<Duration> event;

        /** Local speed cache. */
        private Map<Duration, Speed> cache = new LinkedHashMap<>();

        /**
         * Constructor.
         * @param matrix matrix sign
         * @param detectorType detector type
         * @throws NetworkException when the position is not correct with the lane
         */
        private MatrixDetector(final MatrixSign matrix, final DetectorType detectorType) throws NetworkException
        {
            super(matrix.getId() + "_det", matrix.getLane(), matrix.getLongitudinalPosition(), RelativePosition.FRONT,
                    detectorType);
            this.matrix = matrix;
        }

        @Override
        protected void triggerResponse(final LaneBasedGtu gtu)
        {
            Duration now = gtu.getSimulator().getSimulatorTime();
            this.cache.put(now, gtu.getSpeed());
            check(now);
            // also check at TIME since last vehicle, because no vehicles during TIME means we need to turn off
            if (this.event != null)
            {
                gtu.getSimulator().cancelEvent(this.event);
            }
            Duration later = now.plus(TIME).plus(Duration.ONE); // +1s to prevent floating point issues
            this.event = gtu.getSimulator().scheduleEventAbs(later, () -> check(later));
        }

        /**
         * Checks whether to turn on or off the trigger for this single matrix sign.
         * @param now current time
         */
        private void check(final Duration now)
        {
            // remove all from before TIME ago and keep at most NUMBER in cache
            Duration then = now.minus(TIME);
            while (this.cache.size() > NUMBER || !this.cache.isEmpty() && this.cache.keySet().iterator().next().lt(then))
            {
                Duration first = this.cache.keySet().iterator().next();
                this.cache.remove(first);
            }

            // On: NUMBER vehicles with average speed below ON_SPEED
            if (this.on == null && this.cache.size() == NUMBER && getAverageSpeed().lt(ON_SPEED))
            {
                Logger.ots().trace("Turning ON {}", this.matrix.getFullId());
                this.on = now;
                MatrixControllerRws.this.groupOfSign.get(this.matrix).setOn(this.matrix);
            }
            // Off: at least on for TIME and (no vehicles or 1 through NUMBER vehicles with average speed above OFF_SPEED)
            else if (this.on != null && this.on.lt(then) && (this.cache.isEmpty() || getAverageSpeed().gt(OFF_SPEED)))
            {
                Logger.ots().trace("Turning OFF {}", this.matrix.getFullId());
                this.on = null;
                MatrixControllerRws.this.groupOfSign.get(this.matrix).setOff(this.matrix);
            }
        }

        /**
         * Returns average speed of all vehicles in cache.
         * @return average speed of all vehicles in cache
         */
        private Speed getAverageSpeed()
        {
            return this.cache.entrySet().stream().map((e) -> e.getValue()).reduce(Speed.ZERO, (v1, v2) -> v1.plus(v2))
                    .divide(this.cache.size());
        }

    }

}
