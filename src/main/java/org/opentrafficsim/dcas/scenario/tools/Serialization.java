package org.opentrafficsim.dcas.scenario.tools;

import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Locale;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.base.DoubleScalarRel;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

import nl.tudelft.simulation.dsol.swing.gui.util.Resource;

/**
 * Standard GSON serialization.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public final class Serialization
{

    /** Instance of {@Gson} with all adapters loaded. */
    public static final Gson GSON = new GsonBuilder()
            .registerTypeAdapter(Length.class, new UnitAdapter<>((jr) -> Length.valueOf(jr.nextString())))
            .registerTypeAdapter(Duration.class, new UnitAdapter<>((jr) -> Duration.valueOf(jr.nextString())))
            .registerTypeAdapter(Speed.class, new UnitAdapter<>((jr) -> Speed.valueOf(jr.nextString())))
            .registerTypeAdapter(Acceleration.class, new UnitAdapter<>((jr) -> Acceleration.valueOf(jr.nextString()))).create();

    /** Constructor. */
    private Serialization()
    {
        // utility class
    }

    /**
     * Returns deserialized instance of class from reader.
     * @param <T> type class
     * @param reader reader
     * @param clazz type class
     * @return deserialized instance of class from reader
     */
    public static <T> T fromJsonReader(final JsonReader reader, final Class<T> clazz)
    {
        return GSON.fromJson(reader, clazz);
    }

    /**
     * Returns deserialized instance of class from resource file (in src/main/resources).
     * @param <T> type class
     * @param resource resource file compliant to {@link Resource#getResourceAsStream(String)}. For plain files this should
     *            start with {@code "/"}.
     * @param clazz type class
     * @return deserialized instance of class from resource file
     */
    public static <T> T fromJsonResource(final String resource, final Class<T> clazz)
    {
        return fromJsonReader(new JsonReader(new InputStreamReader(Resource.getResourceAsStream(resource))), clazz);
    }

    /**
     * Function that allows throwing IOException.
     * @param <T> the type of the input to the function
     * @param <R> the type of the result of the function
     */
    private interface IoExceptionTrowingFunction<T, R>
    {
        /**
         * Applies this function to the given argument.
         * @param t the function argument
         * @return the function result
         * @throws IOException IO exception
         */
        R apply(T t) throws IOException;
    }

    /**
     * Generic adapter for all unit types.
     * @param <T> unit type
     */
    private static class UnitAdapter<T extends DoubleScalarRel<?, ?>> extends TypeAdapter<T>
    {
        /** Value reader. */
        private final IoExceptionTrowingFunction<JsonReader, T> reader;

        /**
         * Constructor.
         * @param reader value reader
         */
        UnitAdapter(final IoExceptionTrowingFunction<JsonReader, T> reader)
        {
            this.reader = reader;
        }

        @Override
        public void write(final JsonWriter out, final T value) throws IOException
        {
            // DJUNITS 6.0.1 has much better support for formatting
            Locale locale = Locale.getDefault();
            Locale.setDefault(Locale.US); // use . decimal point
            out.value(value.toString());
            Locale.setDefault(locale);
        }

        @Override
        public T read(final JsonReader in) throws IOException
        {
            return this.reader.apply(in);
        }
    }

}
