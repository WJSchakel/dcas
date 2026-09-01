package org.opentrafficsim.dcas.scenario.tools;

import java.awt.Color;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import org.djunits.unit.Unit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.base.DoubleScalarRel;
import org.djutils.eval.Eval;
import org.djutils.exceptions.Throw;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.reflection.ClassUtil;
import org.opentrafficsim.animation.Colors;
import org.opentrafficsim.animation.colorer.Colorer;
import org.opentrafficsim.animation.data.gtu.GtuTypeGtuColorer;
import org.opentrafficsim.animation.gtu.DefaultCarAnimation.GtuData.GtuMarker;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.logger.Logger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.core.definitions.Definitions;
import org.opentrafficsim.core.distributions.ConstantSupplier;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.object.DetectorType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.dcas.tactical.ChannelTaskToc;
import org.opentrafficsim.dcas.tactical.Dcas;
import org.opentrafficsim.dcas.tactical.DcasFunctionInfrastructure;
import org.opentrafficsim.dcas.tactical.DcasTacticalPlanner;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBias;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBiases;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.gtu.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.FullerImplementation;
import org.opentrafficsim.road.gtu.tactical.lmrs.LmrsFactory.Setting;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.XmlParserException;
import org.opentrafficsim.road.network.factory.xml.parser.DefinitionsParser;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.factory.xml.utils.ParseDistribution;
import org.opentrafficsim.road.network.factory.xml.utils.ParseUtil;
import org.opentrafficsim.road.od.OdApplier;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.road.od.OdOptions;
import org.opentrafficsim.swing.gui.OtsSimulationPanelDecorator;
import org.opentrafficsim.xml.generated.ConstantDistType;
import org.opentrafficsim.xml.generated.GtuTemplates;
import org.opentrafficsim.xml.generated.Ots;

import nl.tudelft.simulation.dsol.experiment.StreamInformation;
import nl.tudelft.simulation.jstats.distributions.DistNormalTrunc;
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
     * @throws ParameterException when a parameter is wrongly set
     */
    public static void applyOd(final RoadNetwork network, final XmlParser xmlParser, final Definitions definitions,
            final OdMatrix odMatrix) throws ParameterException
    {
        ImmutableMap<String, GtuType> gtuTypes = definitions.getAll(GtuType.class);
        GtuType car = gtuTypes.get("CAR");
        GtuType dcas = gtuTypes.get("DCAS");
        GtuType truck = gtuTypes.get("TRUCK");

        DetectorType detectorType = definitions.get(DetectorType.class, "VEHICLE").get();

        StreamInformation streams = network.getSimulator().getModel().getStreamInformation();
        StreamInterface stream = streams.getStream("generation");
        LmrsFactory<?> lmrsFactory =
                new LmrsFactory<>(List.of(car, dcas, truck), List.of(Lmrs::new, DcasTacticalPlanner::new, Lmrs::new))
                        .setStream(stream).set(Setting.ACCELERATION_TRAFFIC_LIGHTS, true)
                        .set(Setting.ACCELERATION_SPEED_LIMIT_TRANSITION, true)
                        .set(Setting.FULLER_IMPLEMENTATION, FullerImplementation.ATTENTION_MATRIX);

        // DCAS parameters
        setDefaultParameter(lmrsFactory, dcas, ChannelTaskToc.TD_TOC);
        setDefaultParameter(lmrsFactory, dcas, Dcas.X_NETWORK); // other parameters in Dcas are only used internally
        setDefaultParameter(lmrsFactory, dcas, DcasFunctionInfrastructure.X_LC);
        setDefaultParameter(lmrsFactory, dcas, DcasFunctionInfrastructure.X_TOC);
        setDefaultParameter(lmrsFactory, dcas, DcasFunctionInfrastructure.X_MRM);
        setDefaultParameter(lmrsFactory, dcas, DcasTacticalPlanner.TAU_STIM);

        OdOptions odOptions = new OdOptions();
        applyToOdOptions(xmlParser, definitions, streams, odOptions, lmrsFactory);
        OdApplier.applyOd(network, odMatrix, odOptions, detectorType);
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
    public static <T extends DoubleScalarRel<U, T>, U extends Unit<U>> ContinuousDistDoubleScalar.Rel<T, U> parseContinuousDist(
            final StreamInformation streams, final ConstantDistType distribution, final U unit, final Eval eval)
            throws XmlParserException
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
        };
    }
}
