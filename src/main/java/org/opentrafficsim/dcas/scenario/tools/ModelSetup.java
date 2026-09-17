package org.opentrafficsim.dcas.scenario.tools;

import java.awt.Color;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.djunits.unit.SpeedUnit;
import org.djunits.unit.Unit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.base.DoubleScalarRel;
import org.djutils.draw.point.Point2d;
import org.djutils.eval.Eval;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.exceptions.Throw;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.reflection.ClassUtil;
import org.opentrafficsim.animation.Colors;
import org.opentrafficsim.animation.colorer.Colorer;
import org.opentrafficsim.animation.data.gtu.AttentionGtuColorer;
import org.opentrafficsim.animation.data.gtu.DesiredHeadwayGtuColorer;
import org.opentrafficsim.animation.data.gtu.DesiredSpeedGtuColorer;
import org.opentrafficsim.animation.data.gtu.GtuTypeGtuColorer;
import org.opentrafficsim.animation.data.gtu.SocialPressureGtuColorer;
import org.opentrafficsim.animation.data.gtu.SplitGtuColorer;
import org.opentrafficsim.animation.data.gtu.SynchronizationGtuColorer;
import org.opentrafficsim.animation.data.gtu.TaskSaturationGtuColorer;
import org.opentrafficsim.animation.gtu.DefaultCarAnimation.GtuData.GtuMarker;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.geometry.FractionalProjectionHelper.FractionalFallback;
import org.opentrafficsim.base.logger.Logger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Definitions;
import org.opentrafficsim.core.distributions.ConstantSupplier;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.object.DetectorType;
import org.opentrafficsim.core.object.LocatedObject;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.dcas.object.matrix.AnimationMatrixData;
import org.opentrafficsim.dcas.object.matrix.MatrixSign;
import org.opentrafficsim.dcas.object.matrix.MatrixSignAnimation;
import org.opentrafficsim.dcas.tactical.ChannelTaskToc;
import org.opentrafficsim.dcas.tactical.Dcas;
import org.opentrafficsim.dcas.tactical.DcasFunctionInfrastructure;
import org.opentrafficsim.dcas.tactical.DcasTacticalPlanner;
import org.opentrafficsim.dcas.tactical.IncentiveKeepFix;
import org.opentrafficsim.dcas.tactical.IncentiveStayOnSlowLanesFix;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBias;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBiases;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.gtu.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.tactical.lmrs.AbstractIncentivesTacticalPlanner;
import org.opentrafficsim.road.gtu.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.FullerImplementation;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.IdmPlusMultiFunction;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.Setting;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.TacticalPlannerProvider;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.tactical.util.lmrs.VoluntaryIncentive;
import org.opentrafficsim.road.network.CrossSectionLink;
import org.opentrafficsim.road.network.Lane;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.XmlParserException;
import org.opentrafficsim.road.network.factory.xml.parser.DefinitionsParser;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.factory.xml.utils.ParseDistribution;
import org.opentrafficsim.road.network.factory.xml.utils.ParseUtil;
import org.opentrafficsim.road.od.OdApplier;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.road.od.OdOptions;
import org.opentrafficsim.swing.gui.OtsSimulationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationPanelDecorator;
import org.opentrafficsim.xml.generated.ConstantDistType;
import org.opentrafficsim.xml.generated.GtuTemplates;
import org.opentrafficsim.xml.generated.Ots;

import nl.tudelft.simulation.dsol.animation.Locatable;
import nl.tudelft.simulation.dsol.animation.d2.Renderable2d;
import nl.tudelft.simulation.dsol.experiment.StreamInformation;
import nl.tudelft.simulation.jstats.distributions.DistEmpiricalDiscreteDouble;
import nl.tudelft.simulation.jstats.distributions.DistNormalTrunc;
import nl.tudelft.simulation.jstats.distributions.empirical.DiscreteEmpiricalDistribution;
import nl.tudelft.simulation.jstats.streams.StreamInterface;

/**
 * Takes an {@link XmlParser} that has already build its information, and re-parses certain internal definitions that are not
 * available through the parser itself. In particular this involves {@link LaneBias} and {@link GtuTemplate} (the latter of
 * which contains the partial information that eventually should be used for a {@link LaneBasedGtuTemplate} which also includes
 * the model factory). Finally this information, together with a provided model factory, are set in {@link OdOptions}.
 * <p>
 * Note that lane biases are mapped to their GTU type ID, while GTU templates are mapped to the GTU template ID. The latter may
 * or may not be equal to the GTU type ID of the referenced GTU type. Copyright (c) 2026-2026 Delft University of Technology, PO
 * Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public final class ModelSetup
{

    /**
     * Constructor.
     */
    private ModelSetup()
    {
        //
    }

    /**
     * Apply OD matrix by setting up vehicle generators with DCAS simulation model factories.
     * @param network network
     * @param xmlParser XML parser
     * @param definitions parsed definitions
     * @param odMatrix OD matrix
     * @param scenarioDcasSettings logic that sets scenario specific DCAS settings
     * @return parameter factory for custom scenario parameters
     * @throws ParameterException when a parameter is wrongly set
     */
    public static ParameterFactoryByType applyOd(final RoadNetwork network, final XmlParser xmlParser,
            final Definitions definitions, final OdMatrix odMatrix, final Consumer<ParameterFactoryByType> scenarioDcasSettings)
            throws ParameterException
    {
        ImmutableMap<String, GtuType> gtuTypes = definitions.getAll(GtuType.class);
        GtuType car = gtuTypes.get("CAR");
        GtuType dcas = gtuTypes.get("DCAS");
        GtuType dcas1 = gtuTypes.get("DCAS1");
        GtuType dcas2 = gtuTypes.get("DCAS2");
        GtuType truck = gtuTypes.get("TRUCK");

        DetectorType detectorType = definitions.get(DetectorType.class, "VEHICLE").get();

        ParameterFactoryByType dcasSettingsFactory = getDcasSettingsFactory(scenarioDcasSettings);
        TacticalPlannerProvider<AbstractIncentivesTacticalPlanner> dcasTacticalPlannerFactory =
                DcasTacticalPlanner.factory(dcasSettingsFactory);

        StreamInformation streams = network.getSimulator().getModel().getStreamInformation();
        StreamInterface stream = streams.getStream("generation");
        Set<Supplier<VoluntaryIncentive>> setTruck =
                Set.of(() -> IncentiveKeepFix.SINGLETON, () -> IncentiveStayOnSlowLanesFix.SINGLETON);
        Set<Supplier<VoluntaryIncentive>> setCar = Set.of(() -> IncentiveKeepFix.SINGLETON);
        Boolean socio = true;
        LmrsFactory<AbstractIncentivesTacticalPlanner> lmrsFactory = new LmrsFactory<>(List.of(car, dcas1, dcas2, truck),
                List.of(Lmrs::new, dcasTacticalPlannerFactory, dcasTacticalPlannerFactory, Lmrs::new))
                        .set(Setting.CAR_FOLLOWING_MODEL, IdmPlusMultiFunction.SINGLETON).setStream(stream)
                        .set(Setting.ACCELERATION_TRAFFIC_LIGHTS, true).set(Setting.ACCELERATION_SPEED_LIMIT_TRANSITION, true)
                        .set(Setting.FULLER_IMPLEMENTATION, FullerImplementation.ATTENTION_MATRIX)
                        .set(Setting.CUSTOM_VOLUNTARY_INCENTIVES, setCar, car)
                        .set(Setting.CUSTOM_VOLUNTARY_INCENTIVES, setCar, dcas1)
                        .set(Setting.CUSTOM_VOLUNTARY_INCENTIVES, setCar, dcas2)
                        .set(Setting.CUSTOM_VOLUNTARY_INCENTIVES, setTruck, truck).set(Setting.INCENTIVE_KEEP, false)
                        .set(Setting.SOCIO_PRESSURE, socio).set(Setting.SOCIO_TAILGATING, socio).set(Setting.SOCIO_SPEED, socio)
                        .set(Setting.SOCIO_LANE_CHANGE, socio).set(Setting.SYNCHRONIZATION, Synchronization.ALIGN_GAP)
                        .set(Setting.ACCELERATION_NO_SLOW_LANE_OVERTAKE, true);
        // .set(Setting.FRACTION_OVERESTIMATION, Assumptions.get().fOverEst()); Bug: gets overwritten by default
        fOverEstFix(lmrsFactory, stream); // Solution: create custom distributions

        // DCAS behavioral human parameters
        setDefaultParameter(lmrsFactory, dcas, ChannelTaskToc.TD_TOC_LOW);
        setDefaultParameter(lmrsFactory, dcas, ChannelTaskToc.TD_TOC_HIGH);
        setDefaultParameter(lmrsFactory, dcas, DcasTacticalPlanner.TAU_STIM);

        // regular human parameters
        lmrsFactory.addParameter(ChannelFuller.TAU_MIN, Assumptions.get().human().tauMin());
        lmrsFactory.addParameter(ChannelFuller.TAU_MAX, Assumptions.get().human().tauMax());
        lmrsFactory.addParameter(truck, ParameterTypes.A, Acceleration.ofSI(0.8));

        OdOptions odOptions = new OdOptions();
        applyToOdOptions(xmlParser, definitions, streams, odOptions, lmrsFactory);
        OdApplier.applyOd(network, odMatrix, odOptions, detectorType);

        return lmrsFactory;
    }

    /**
     * Returns DCAS settings factory.
     * @param scenarioDcasSettings scenario specific DCAS settings
     * @return DCAS settings factory
     * @throws ParameterException when the set value does not comply with the type
     */
    private static ParameterFactoryByType getDcasSettingsFactory(final Consumer<ParameterFactoryByType> scenarioDcasSettings)
            throws ParameterException
    {
        /*
         * Note on B values: There is the regular B from the IDM, applied in car-following, and as threshold for
         * synchronization. The value of B0 applies as a deceleration limit in the free acceleration term when DCAS is faster
         * than the current target speed. This value is also used for stopping during an Minimum Risk Maneuver. Finally there is
         * B_MAX, which is the maximum deceleration the system will allow. If the calculated deceleration is stronger, it will
         * be limited and a Transition Of Control request follows.
         */
        ParameterFactoryByType dcasSettings = new ParameterFactoryByType();

        dcasSettings.addParameter(ParameterTypes.S0, Assumptions.get().dcas().cf().s0Dcas());
        dcasSettings.addParameter(ParameterTypes.T, Assumptions.get().dcas().cf().TDcas());
        dcasSettings.addParameter(ParameterTypes.A, Assumptions.get().dcas().cf().aDcas());
        dcasSettings.addParameter(ParameterTypes.B, Assumptions.get().dcas().cf().bDcas());
        dcasSettings.addParameter(ParameterTypes.B0, Assumptions.get().dcas().cf().b0Dcas());
        dcasSettings.addParameter(AbstractIdm.DELTA, Assumptions.get().dcas().cf().deltaDcas());
        setDefaultParameter(dcasSettings, Dcas.X_NETWORK);
        setDefaultParameter(dcasSettings, Dcas.MAX_B_DCAS);
        setDefaultParameter(dcasSettings, Dcas.MIN_TTC_DCAS);
        setDefaultParameter(dcasSettings, Dcas.MIN_T_DCAS);
        setDefaultParameter(dcasSettings, Dcas.DT_DCAS);
        setDefaultParameter(dcasSettings, Dcas.LC_DCAS);
        setDefaultParameter(dcasSettings, Dcas.SHOULDER_DCAS);
        setDefaultParameter(dcasSettings, Dcas.TOC_ESCALATE);
        setDefaultParameter(dcasSettings, DcasFunctionInfrastructure.X_LC);
        setDefaultParameter(dcasSettings, DcasFunctionInfrastructure.X_TOC);
        setDefaultParameter(dcasSettings, DcasFunctionInfrastructure.X_MRM);

        return dcasSettings;
    }

    /**
     * Sets the right parameter distribution to adhere to fOverEst.
     * @param lmrsFactory factory
     * @param stream stream
     */
    private static void fOverEstFix(final LmrsFactory<?> lmrsFactory, final StreamInterface stream)
    {
        double fOverEst = Assumptions.get().scenario().fOverEst();
        if (fOverEst == 0.0)
        {
            lmrsFactory.addParameter(Fuller.OVER_EST, -1.0);
        }
        else if (fOverEst == 1.0)
        {
            lmrsFactory.addParameter(Fuller.OVER_EST, 1.0);
        }
        else
        {
            lmrsFactory.addParameter(Fuller.OVER_EST, new DistEmpiricalDiscreteDouble(stream,
                    new DiscreteEmpiricalDistribution(new Double[] {-1.0, 1.0}, new double[] {1.0 - fOverEst, 1.0})));
        }
    }

    /**
     * Sets default parameter in parameter factory.
     * @param <T> type of parameter value
     * @param parameterFactory parameter factory
     * @param parameterType parameter type
     * @throws ParameterException when the set value does not comply with the type
     */
    private static <T> void setDefaultParameter(final ParameterFactoryByType parameterFactory,
            final ParameterType<T> parameterType) throws ParameterException
    {
        parameterFactory.addParameter(parameterType, parameterType.getDefaultValue());
    }

    /**
     * Sets default parameter in parameter factory.
     * @param <T> type of parameter value
     * @param parameterFactory parameter factory
     * @param gtuType GTU type for which the parameter applies
     * @param parameterType parameter type
     * @throws ParameterException when the set value does not comply with the type
     */
    private static <T> void setDefaultParameter(final ParameterFactoryByType parameterFactory, final GtuType gtuType,
            final ParameterType<T> parameterType) throws ParameterException
    {
        parameterFactory.addParameter(gtuType, parameterType, parameterType.getDefaultValue());
    }

    /**
     * Takes an {@link XmlParser} that has already build its information, and re-parses certain internal definitions that are
     * not available through the parser itself. In particular this involves {@link LaneBias} and {@link GtuTemplate} (the latter
     * of which contains the partial information that eventually should be used for a {@link LaneBasedGtuTemplate} which also
     * includes the model factory). Finally this information, together with a provided model factory, are set in
     * {@link OdOptions}.
     * <p>
     * Note that lane biases are mapped to their GTU type ID, while GTU templates are mapped to the GTU template ID. The latter
     * may or may not be equal to the GTU type ID of the referenced GTU type.
     * @param parser parser, should already have parsed its information
     * @param definitions as returned by the {@link XmlParser#build} method
     * @param streams random number stream information as parsed and obtained from the model
     * @param odOptions {@link OdOptions} in to which {@link OdOptions#LANE_BIAS} and {@link OdOptions#GTU_TYPE} are set
     * @param lmrsFactory tactical model factory
     */
    private static void applyToOdOptions(final XmlParser parser, final Definitions definitions, final StreamInformation streams,
            final OdOptions odOptions, final LmrsFactory<?> lmrsFactory)
    {
        Ots otsTag;
        Eval eval;
        try
        {
            Field otsField = ClassUtil.resolveField(parser, "ots");
            otsField.setAccessible(true);
            otsTag = (Ots) otsField.get(parser);

            Field evalField = ClassUtil.resolveField(parser, "eval");
            evalField.setAccessible(true);
            eval = (Eval) evalField.get(parser);
        }
        catch (NoSuchFieldException | IllegalArgumentException | IllegalAccessException ex)
        {
            throw new OtsRuntimeException(ex);
        }
        Throw.when(otsTag.getDefinitions() == null, IllegalArgumentException.class, "No definitions in XML.");

        odOptions.set(OdOptions.LANE_BIAS, parseLaneBiases(otsTag, eval, definitions));
        try
        {
            // Note: lmrsFactory is both a LaneBasedTacticalPlannerFactory and a ParameterFactory
            DefaultLaneBasedGtuCharacteristicsGeneratorOd characteristicsGenerator =
                    new Factory(new LaneBasedStrategicalRoutePlannerFactory(lmrsFactory, lmrsFactory))
                            .setTemplates(parseGtuTemplates(otsTag, eval, definitions, streams)).create();
            odOptions.set(OdOptions.GTU_TYPE, characteristicsGenerator);
        }
        catch (XmlParserException ex)
        {
            throw new OtsRuntimeException(ex);
        }
    }

    /**
     * Parses {@link LaneBias} from the main OTS tag.
     * @param otsTag main OTS tag
     * @param eval expression evaluator
     * @param definitions as returned by the {@link XmlParser#build} method
     * @return {@link Map} of GTU type ID vs. parsed {@link LaneBias}
     */
    private static LaneBiases parseLaneBiases(final Ots otsTag, final Eval eval, final Definitions definitions)
    {
        LaneBiases laneBiases = new LaneBiases();
        for (org.opentrafficsim.xml.generated.LaneBiases laneBiasTags : ParseUtil.getObjectsOfType(
                otsTag.getDefinitions().getIncludeAndGtuTypesAndGtuTemplates(),
                org.opentrafficsim.xml.generated.LaneBiases.class))
        {
            for (org.opentrafficsim.xml.generated.LaneBias laneBiasTag : laneBiasTags.getLaneBias())
            {
                LaneBias laneBias = DefinitionsParser.parseLaneBias(laneBiasTag, eval);
                String gtuTypeId = laneBiasTag.getGtuType().get(eval);
                laneBiases.addBias(definitions.get(GtuType.class, gtuTypeId)
                        .orElseThrow(() -> new IllegalStateException("GTU type " + gtuTypeId + " is not defined.")), laneBias);
            }
        }
        return laneBiases;
    }

    /**
     * Parses {@link GtuTemplate} from the main OTS tag.
     * @param otsTag main OTS tag
     * @param eval expression evaluator
     * @param definitions as returned by the {@link XmlParser#build} method
     * @param streams random number stream information as parsed and obtained from the model
     * @return {@link Set} of {@link GtuTemplate}
     * @throws XmlParserException when required information is not specified
     */
    private static Set<GtuTemplate> parseGtuTemplates(final Ots otsTag, final Eval eval, final Definitions definitions,
            final StreamInformation streams) throws XmlParserException
    {
        Map<GtuType, GtuTemplate> map = new LinkedHashMap<>();
        for (GtuTemplates gtuTemplateTags : ParseUtil
                .getObjectsOfType(otsTag.getDefinitions().getIncludeAndGtuTypesAndGtuTemplates(), GtuTemplates.class))
        {
            for (org.opentrafficsim.xml.generated.GtuTemplate gtuTemplateTag : gtuTemplateTags.getGtuTemplate())
            {
                GtuType gtuType = definitions.getOrThrow(GtuType.class, gtuTemplateTag.getGtuType().get(eval));
                Supplier<Length> lengthDist = parseContinuousDist(streams, gtuTemplateTag.getLengthDist(),
                        gtuTemplateTag.getLengthDist().getLengthUnit().get(eval), eval);
                Supplier<Length> widthDist = parseContinuousDist(streams, gtuTemplateTag.getWidthDist(),
                        gtuTemplateTag.getWidthDist().getLengthUnit().get(eval), eval);
                Supplier<Speed> maxSpeedDist = parseContinuousDist(streams, gtuTemplateTag.getMaxSpeedDist(),
                        gtuTemplateTag.getMaxSpeedDist().getSpeedUnit().get(eval), eval);
                // accelerations are optional, if not specified, let's follow defaults in GtuTemplate constructor
                Supplier<Acceleration> maxAccelerationDist = gtuTemplateTag.getMaxAccelerationDist() != null
                        ? parseContinuousDist(streams, gtuTemplateTag.getMaxSpeedDist(),
                                gtuTemplateTag.getMaxAccelerationDist().getAccelerationUnit().get(eval), eval)
                        : new ConstantSupplier<>(Acceleration.ofSI(3.0));
                Supplier<Acceleration> maxDecelerationDist = gtuTemplateTag.getMaxDecelerationDist() != null
                        ? parseContinuousDist(streams, gtuTemplateTag.getMaxSpeedDist(),
                                gtuTemplateTag.getMaxDecelerationDist().getAccelerationUnit().get(eval), eval)
                        : new ConstantSupplier<>(Acceleration.ofSI(-8.0));
                GtuTemplate gtuTemplate =
                        new GtuTemplate(gtuType, lengthDist, widthDist, maxSpeedDist, maxAccelerationDist, maxDecelerationDist);
                map.put(gtuType, gtuTemplate);
            }
        }
        /*
         * For GtuTypes with no GTU template defined, loop parent types until a GTU template is found. Then store it for the
         * original GTU type.
         */
        for (GtuType gtuType : definitions.getAll(GtuType.class).values())
        {
            GtuType parent = gtuType;
            while (parent != null && !map.containsKey(parent))
            {
                parent = parent.getParent().orElse(null);
            }
            if (parent != null)
            {
                map.put(gtuType, map.get(parent).copyForGtuType(gtuType));
            }
            else
            {
                Logger.ots().trace("No GtuTemplate for " + gtuType + " as none is defined, nor for any of the parent types.");
            }
        }
        return new LinkedHashSet<>(map.values());
    }

    /**
     * Parse a relative unit distribution, e.g. <code>UNIFORM(1, 3) m</code>.
     * @param streams the map with streams from the RUN tag
     * @param distribution the tag to parse, a sub type of ConstantDistType
     * @param unit unit
     * @param eval expression evaluator.
     * @param <T> value type
     * @param <U> unit type
     * @return a typed continuous random distribution.
     * @throws XmlParserException in case of a parse error.
     */
    private static <T extends DoubleScalarRel<U, T>,
            U extends Unit<U>> ContinuousDistDoubleScalar.Rel<T, U> parseContinuousDist(final StreamInformation streams,
                    final ConstantDistType distribution, final U unit, final Eval eval) throws XmlParserException
    {
        if (distribution.getNormalTrunc() != null)
        {
            // Bug: ParseDistribution.parseContinuousDist() has "else if (distType.getNormal() != null)" for DistNormalTrunc
            StreamInterface stream = ParseUtil.findStream(streams, distribution.getRandomStream(), eval);
            DistNormalTrunc dist = new DistNormalTrunc(stream, distribution.getNormalTrunc().getMu().get(eval),
                    distribution.getNormalTrunc().getSigma().get(eval), distribution.getNormalTrunc().getMin().get(eval),
                    distribution.getNormalTrunc().getMax().get(eval));
            return new ContinuousDistDoubleScalar.Rel<T, U>(dist, unit);
        }
        return ParseDistribution.parseContinuousDist(streams, distribution, unit, eval);
    }

    /**
     * Returns decorator with standard features and DCAS features.
     * @param gtuTypes GTU types
     * @return decorator with standard features and DCAS features
     */
    public static OtsSimulationPanelDecorator getDecorator(final ImmutableMap<String, GtuType> gtuTypes)
    {
        return new OtsSimulationPanelDecorator()
        {
            @Override
            public List<Colorer<? super Gtu>> getGtuColorers()
            {
                List<Colorer<? super Gtu>> colorers = new ArrayList<>(DEFAULT_GTU_COLORERS);

                Map<GtuType, Color> colors = new LinkedHashMap<>();
                gtuTypes.forEach((id, g) ->
                {
                    switch (id)
                    {
                        case "CAR":
                            colors.put(g, Colors.OTS_BLUE);
                            return;
                        case "DCAS":
                            colors.put(g,
                                    new Color(Colors.OTS_BLUE.getBlue(), Colors.OTS_BLUE.getGreen(), Colors.OTS_BLUE.getRed())
                                            .brighter());
                            return;
                        case "TRUCK":
                            colors.put(g, Color.WHITE);
                            return;
                        default:
                            return;
                    }
                });

                colorers.add(new GtuTypeGtuColorer(colors, Color.CYAN));
                colorers.add(new DcasStateColorer());
                colorers.add(new SynchronizationGtuColorer());
                colorers.add(new SocialPressureGtuColorer());
                colorers.add(new TaskSaturationGtuColorer());
                colorers.add(new AttentionGtuColorer());
                colorers.add(new DesiredSpeedGtuColorer(new Speed(70.0, SpeedUnit.KM_PER_HOUR),
                        new Speed(140.0, SpeedUnit.KM_PER_HOUR)));
                colorers.add(new DesiredHeadwayGtuColorer(Duration.ofSI(0.5), Duration.ofSI(1.6)));
                colorers.add(new SplitGtuColorer());

                return colorers;
            }

            @Override
            public Map<GtuType, GtuMarker> getGtuMarkers()
            {
                return new LinkedHashMap<>()
                {
                    {
                        if (gtuTypes.containsKey("TRUCK"))
                        {
                            put(gtuTypes.get("TRUCK"), GtuMarker.SQUARE);
                        }
                    }
                };
            }

            @Override
            public void animateSimulation(final OtsSimulationPanel simulationPanel, final Network network)
            {
                OtsSimulationPanelDecorator.super.animateSimulation(simulationPanel, network);

                Map<Locatable, Renderable2d<?>> animatedMatrixSigns = new LinkedHashMap<Locatable, Renderable2d<?>>();
                EventListener matrixAnimator = new EventListener()
                {
                    @Override
                    public void notify(final Event event)
                    {
                        LocatedObject object = network.getObjectMap().get((String) event.getContent());
                        if (object instanceof MatrixSign matrix)
                        {
                            if (event.getType().equals(Network.OBJECT_ADD_EVENT))
                            {
                                animatedMatrixSigns.put(object,
                                        new MatrixSignAnimation(new AnimationMatrixData(matrix), network.getSimulator()));
                            }
                            else
                            {
                                // OBJECT_REMOVE_EVENT
                                Renderable2d<?> renderable = animatedMatrixSigns.remove(object);
                                if (renderable != null)
                                {
                                    renderable.destroy(network.getSimulator());
                                }
                            }
                        }
                    }
                };
                network.addListener(matrixAnimator, Network.OBJECT_ADD_EVENT);
                network.addListener(matrixAnimator, Network.OBJECT_REMOVE_EVENT);

                // current objects
                for (LocatedObject object : network.getObjectMap().values())
                {
                    if (object instanceof MatrixSign matrix)
                    {
                        animatedMatrixSigns.put(object,
                                new MatrixSignAnimation(new AnimationMatrixData(matrix), network.getSimulator()));
                    }
                }
            }

        };
    }

    /**
     * Adds matrix signs on each lane at the given position on the link.
     * @param link link
     * @param position position
     * @throws NetworkException if the position is out of bounds of the link
     */
    public static void addMatrixGantry(final Link link, final Length position) throws NetworkException
    {
        Point2d p = link.getDesignLine().getLocation(position);
        for (Lane lane : ((CrossSectionLink) link).getLanes())
        {
            double pos = lane.getCenterLine().projectFractionalAt(link.getStartNode().getHeading(),
                    link.getEndNode().getHeading(), p.x, p.y, FractionalFallback.ENDPOINT);
            new MatrixSign("Matrix_" + position, lane, Length.ofSI(pos * lane.getCenterLine().getLength()));
        }
    }

}
