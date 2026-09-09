package org.opentrafficsim.dcas.scenario.tools;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;

/**
 * Data container for all assumptions. Any changes to this class need to be reflected in the resource file
 * {@code assumptions.json}.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @param scenario scenario assumptions
 * @param dcas DCAS assumptions
 * @param human human assumptions
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public record Assumptions(Scenario scenario, Dcas dcas, Human human)
{

    /** Singleton instance returned by {@code get()}. */
    private static final Assumptions ASSUMPTIONS = Serialization.fromJsonResource("/assumptions.json", Assumptions.class);

    /**
     * Returns instance from JSON file. This method may be called often as it returns a cached version.
     * @return instance from JSON file
     */
    public static Assumptions get()
    {
        return ASSUMPTIONS;
    }

    /**
     * Scenario assumptions.
     * @param penetrationLow low DCAS penetration rate (vehicle has DCAS)
     * @param penetrationHigh high DCAS penetration rate (vehicle has DCAS)
     * @param activationRate rate of activation on vehicles with DCAS
     * @param tocNonResponseRate DCAS driver transition-of-control non-response rate
     * @param fOverEst fraction of drivers that over-estimates stimuli (safer for speed difference, less safe for distance)
     */
    public record Scenario(double penetrationLow, double penetrationHigh, double activationRate, double tocNonResponseRate,
            double fOverEst)
    {
    }

    /**
     * DCAS assumptions.
     * @param dtDcas DCAS system time step
     * @param tocEscDcas time after which a Transition Of Control request is escalated
     * @param xNetwork DCAS extent over which it knows the network
     * @param cf car-following assumptions
     * @param lc lane-change settings
     * @param infra infra settings
     */
    public record Dcas(Duration dtDcas, Duration tocEscDcas, Length xNetwork, CarFollowing cf, LaneChange lc, Infra infra)
    {

        /**
         * Car-following assumptions.
         * @param s0Dcas DCAS IDM car-following stopping distance
         * @param TDcas DCAS IDM car-following time headway
         * @param aDcas DCAS IDM car-following acceleration parameter
         * @param bDcas DCAS IDM car-following deceleration parameter
         * @param b0Dcas DCAS IDM car-following adjustment deceleration parameter
         * @param deltaDcas DCAS IDM car-following delta parameter
         * @param maxBDcas DCAS maximum deceleration
         */
        public record CarFollowing(Length s0Dcas, Duration TDcas, Acceleration aDcas, Acceleration bDcas, Acceleration b0Dcas,
                double deltaDcas, Acceleration maxBDcas)
        {
        }

        /**
         * Lane-change assumptions.
         * @param minTtcDcas DCAS LC minimum TTC
         * @param minTDcas DCAS LC minimum time headway
         * @param lcDcas DCAS is able to change lane
         * @param shoulderDcas DCAS is able to perform MRM to the shoulder
         */
        public record LaneChange(Duration minTtcDcas, Duration minTDcas, boolean lcDcas, boolean shoulderDcas)
        {
        }

        /**
         * Infra assumptions.
         * @param infraLc distance remaining per lane change where DCAS starts to change lane
         * @param infraToc distance remaining per lane change where DCAS starts to request a Transition Of Control
         * @param infraMrm distance remaining per lane change where DCAS starts to perform s Minimum Risk Maneuver
         */
        public record Infra(Length infraLc, Length infraToc, Length infraMrm)
        {
        }

    }

    /**
     * Human assumptions.
     * @param tdTocLow task-demand during low level request for Transition Of Control
     * @param tdTocHigh task-demand during high level request for Transition Of Control
     * @param tauStim duration over which a stimulus has to be persistent for action
     * @param tauMin minimum perception delay (default 0.32s)
     * @param tauMax maximum perception delay (default 1.19s)
     */
    public record Human(double tdTocLow, double tdTocHigh, Duration tauStim, Duration tauMin, Duration tauMax)
    {
    }
}
