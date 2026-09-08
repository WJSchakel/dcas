package org.opentrafficsim.dcas.scenario.tools;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;

/**
 * Data container for all assumptions. Any changes to this class need to be reflected in the resource file
 * {@code assumptions.json}.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 * @param penetrationLow low DCAS penetration rate (vehicle has DCAS)
 * @param penetrationHigh high DCAS penetration rate (vehicle has DCAS)
 * @param activationRate rate of activation on vehicles with DCAS
 * @param s0Dcas DCAS IDM car-following stopping distance
 * @param TDcas DCAS IDM car-following time headway
 * @param aDcas DCAS IDM car-following acceleration parameter
 * @param bDcas DCAS IDM car-following deceleration parameter
 * @param b0Dcas DCAS IDM car-following adjustment deceleration parameter
 * @param deltaDcas DCAS IDM car-following delta parameter
 * @param xNetwork DCAS extent over which it knows the network
 * @param maxBDcas DCAS maximum deceleration
 * @param minTtcDcas DCAS LC minimum TTC
 * @param minTDcas DCAS LC minimum time headway
 * @param dtDcas DCAS system time step
 * @param fOverEst fraction of drivers that over-estimates stimuli (safer for speed difference, less safe for distance)
 * @param lcDcas DCAS is able to change lane
 * @param shoulderDcas DCAS is able to perform MRM to the shoulder
 * @param infraLc distance remaining per lane change where DCAS starts to change lane
 * @param infraToc distance remaining per lane change where DCAS starts to request a Transition Of Control
 * @param infraMrm distance remaining per lane change where DCAS starts to perform s Minimum Risk Maneuver
 * @param tdToc task-demand during Transition Of Control request
 * @param tauStim duration over which a stimulus has to be persistent for action
 * @param tauMin minimum perception delay (default 0.32s)
 * @param tauMax maximum perception delay (default 1.19s)
 * @param vGainDcas vGain parameter value for drivers with DCAS activated
 * @param socioDcas socio parameter value for drivers with DCAS activated
 * @param tocNonResponseRate DCAS driver transition-of-control non-response rate
 */
public record Assumptions(double penetrationLow, double penetrationHigh, double activationRate, Length s0Dcas, Duration TDcas,
        Acceleration aDcas, Acceleration bDcas, Acceleration b0Dcas, double deltaDcas, Length xNetwork, Acceleration maxBDcas,
        Duration minTtcDcas, Duration minTDcas, Duration dtDcas, double fOverEst, boolean lcDcas, boolean shoulderDcas,
        Length infraLc, Length infraToc, Length infraMrm, double tdToc, Duration tauStim, Duration tauMin, Duration tauMax,
        Speed vGainDcas, double socioDcas, double tocNonResponseRate)
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

}
