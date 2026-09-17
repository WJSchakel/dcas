package org.opentrafficsim.dcas.object.matrix;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.font.GlyphVector;
import java.awt.font.TextAttribute;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.image.BufferedImage;
import java.awt.image.ImageObserver;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.imageio.ImageIO;

import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.DirectedPoint2d;
import org.djutils.draw.point.Point2d;
import org.opentrafficsim.animation.DrawLevel;
import org.opentrafficsim.animation.OtsRenderable;
import org.opentrafficsim.base.geometry.OtsShape;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.MatrixState;
import org.opentrafficsim.dcas.object.matrix.MatrixSignAnimation.MatrixSignData;

import nl.tudelft.simulation.naming.context.ContextInterface;
import nl.tudelft.simulation.naming.context.Contextualized;
import nl.tudelft.simulation.naming.context.JvmContext;

/**
 * MatrixSignAnimation.java.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class MatrixSignAnimation extends OtsRenderable<MatrixSignData>
{

    /** Shape cache. */
    private static final Map<String, Shape> SHAPE_CACHE = new LinkedHashMap<>();

    /** Standard painting size. */
    private static final double SIZE = 2.0;

    /** Rounded corners. */
    private static final double ROUNDING = SIZE * 0.2;

    /** Edge width. */
    private static final float EDGE_WIDTH = (float) (SIZE * 0.015);

    /** Font size relative to SIZE. */
    private static final double FONT_SIZE = SIZE * 0.66;

    /** Reduced character thickness for led strips to be spaced more closely. */
    private static final float FONT_NARROWING = (float) (SIZE * 0.022);

    /** Tracking for 3-digit numbers (reduces space between digits). */
    private static final float FONT_THREE_DIGIT_TRACKING = -0.07f;

    /** Squeezes the font vertically to slightly better match real matrix sign font based on built in sans serif font. */
    private static final double FONT_Y_SQUEEZE = 0.9;

    /** Distance between LEDs along strip in text contour. */
    private static final double FONT_PITCH = SIZE * 0.035;

    /** Distance between LEDs on straight lines in arrows and cross. */
    private static final double LINE_PITCH = SIZE * 0.03;

    /** Distance between LEDs on end-of-restrictions sign. */
    private static final double END_PITCH = SIZE * 0.025;

    /** Curve flattness to poly line. */
    private static final double FLATNESS = SIZE * 0.001;

    /** Background color. */
    private static final Color BACKGROUND = new Color(15, 15, 15);

    /** Edge color. */
    private static final Color EDGE = new Color(40, 40, 40);

    /** Amber light color. */
    private static final Color AMBER = new Color(255, 180, 0);

    /** Amber light speckle color. */
    private static final Color AMBER_SPECKLE = new Color(120, 80, 0);

    /** Red color. */
    private static final Color RED = new Color(227, 0, 50);

    /** Green color. */
    private static final Color GREEN = new Color(0, 255, 144);

    /** White color. */
    private static final Color WHITE = Color.WHITE;

    /**
     * Constructor.
     * @param source source
     * @param contextProvider context
     */
    public MatrixSignAnimation(final MatrixSignData source, final Contextualized contextProvider)
    {
        super(source, contextProvider);
    }

    @Override
    @SuppressWarnings("leftcurly")
    public void paint(final Graphics2D g, final ImageObserver observer)
    {
        setRendering(g);

        /*
         * Between scales 2.0 and 10.0 we want the scale to snap to 10.0. This creates a range of zoom levels where we accept
         * overlap between adjacent matrix signs, but are still able to see the content. Below a scale of 2.0 the overlap
         * becomes severe for normal lane widths and we default to the normal small scale and so tiny matrix sings, effectively
         * only communicating that there is a matrix sign, rather than its contents. For larger scales we can paint at the
         * normal large scale. The local object Y-scale is used so networks that are zoomed in more in their Y-dimension allow
         * larger matrix signs to be painted. To obtain a separate Y-scale value from the X-scale value, we need to use the
         * default enabled Y-scaling of the parent class. However, we actually want to paint the matrix signs 1:1. We thus scale
         * the X-dimension by scaleY/scaleX. For the case of 2.0 < scaleY < 10.0, the X-dimension is also scaled by 10.0/scaleY,
         * simplifying to 10.0/scaleX.
         */
        AffineTransform at = g.getTransform();
        double scaleObjX = Math.hypot(at.getScaleX(), at.getShearY());
        double scaleObjY = Math.hypot(at.getScaleY(), at.getShearX());
        if (2.0 < scaleObjY && scaleObjY < 10.0)
        {
            g.scale(10.0 / scaleObjX, 10.0 / scaleObjY);
        }
        else
        {
            g.scale(scaleObjY / scaleObjX, 1.0);
        }
        // painting logic is based on top of screen, but for a zero angle "up" is to the right of the screen
        g.rotate(Math.PI / 2.0);

        // housing
        double s = SIZE - ((double) EDGE_WIDTH);
        Shape housing = new RoundRectangle2D.Double(-s / 2.0, -s / 2.0, s, s, ROUNDING, ROUNDING);
        g.setColor(BACKGROUND);
        g.fill(housing);
        g.setColor(EDGE);
        g.setStroke(new BasicStroke(EDGE_WIDTH));
        g.draw(housing);

        switch (getSource().getState())
        {
            case V50 -> drawString(g, "50");
            case V70 -> drawString(g, "70");
            case V80 -> drawString(g, "80");
            case V90 -> drawString(g, "90");
            case V100 -> drawString(g, "100");
            case X -> drawRedCross(g);
            case LEFT -> drawWhiteArrow(g, true);
            case RIGHT -> drawWhiteArrow(g, false);
            case DOWN -> drawGreenDownArrow(g);
            case END -> drawEndRestriction(g);
            case BLANK -> drawAmberLamps(g, false);
            default -> {
                // nothing
            }
        }
        setRendering(g);
    }

    /**
     * Draw string (i.e. a speed number).
     * @param g graphics
     * @param string string
     */
    private static void drawString(final Graphics2D g, final String string)
    {
        drawAmberLamps(g, true);
        // cannot cache shape as this depends on Graphics2D
        Font font = new Font(Font.SANS_SERIF, Font.PLAIN, 12).deriveFont((float) (FONT_SIZE));
        if (string.length() > 2)
        {
            // reduce tracking for e.g. "100"
            Map<TextAttribute, Object> attributes = new LinkedHashMap<>();
            attributes.put(TextAttribute.TRACKING, FONT_THREE_DIGIT_TRACKING);
            font = font.deriveFont(attributes);
        }
        GlyphVector gv = font.createGlyphVector(g.getFontRenderContext(), string);
        Shape outline = gv.getOutline();

        // reduce character thickness for led strips to be spaced more closely
        BasicStroke stroke = new BasicStroke(FONT_NARROWING, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);
        Area reduced = new Area(outline);
        reduced.subtract(new Area(stroke.createStrokedShape(outline)));

        // position text in center
        Rectangle2D b = reduced.getBounds2D();
        AffineTransform at = new AffineTransform();
        at.translate(-b.getCenterX(), -b.getCenterY() * FONT_Y_SQUEEZE);
        at.scale(1.0, FONT_Y_SQUEEZE);
        Shape shape = at.createTransformedShape(reduced);
        drawLedStrip(g, shape, FONT_PITCH, WHITE);
    }

    /**
     * Draw white arrow.
     * @param g graphics
     * @param left left or right arrow
     */
    private static void drawWhiteArrow(final Graphics2D g, final boolean left)
    {
        drawAmberLamps(g, true);
        double dir = left ? -1.0 : 1.0;
        Shape shape = SHAPE_CACHE.computeIfAbsent("arrow_" + (left ? "left" : "right"), (k) ->
        {
            double diagPitch = LINE_PITCH / Math.sqrt(2);
            double box = 0.25; // box size, the arrow head and line-start are at the box edge
            double line = 0.1; // line-end
            double head = 0.2; // arrow head end points
            Path2D p = new Path2D.Double();
            for (int i = -1; i < 2; i++)
            {
                p.moveTo(dir * -box * SIZE + dir * i * diagPitch, -box * SIZE - i * diagPitch);
                p.lineTo(dir * line * SIZE + dir * i * diagPitch, line * SIZE - i * diagPitch);
                p.moveTo(dir * box * SIZE + dir * i * LINE_PITCH, -head * SIZE);
                p.lineTo(dir * box * SIZE + dir * i * LINE_PITCH, box * SIZE + i * LINE_PITCH);
                p.lineTo(dir * -head * SIZE, box * SIZE + i * LINE_PITCH);
            }
            return p;
        });
        drawLedStrip(g, shape, LINE_PITCH, Color.WHITE);
    }

    /**
     * Draw red cross.
     * @param g graphics
     */
    private static void drawRedCross(final Graphics2D g)
    {
        drawAmberLamps(g, false);
        Shape shape = SHAPE_CACHE.computeIfAbsent("cross", (k) ->
        {
            double diagPitch = LINE_PITCH / Math.sqrt(2);
            double r0 = SIZE * 0.045; // inner line end points
            double r1 = SIZE * 0.34; // outer line end points
            Path2D p = new Path2D.Double();
            for (int i = -1; i < 2; i++)
            {
                p.moveTo(-r0 + i * diagPitch, -r0 - i * diagPitch);
                p.lineTo(-r1 + i * diagPitch, -r1 - i * diagPitch);
                p.moveTo(-r0 + i * diagPitch, r0 + i * diagPitch);
                p.lineTo(-r1 + i * diagPitch, r1 + i * diagPitch);
                p.moveTo(r0 - i * diagPitch, -r0 - i * diagPitch);
                p.lineTo(r1 - i * diagPitch, -r1 - i * diagPitch);
                p.moveTo(r0 - i * diagPitch, r0 + i * diagPitch);
                p.lineTo(r1 - i * diagPitch, r1 + i * diagPitch);
            }
            return p;
        });
        drawLedStrip(g, shape, LINE_PITCH, RED);
    }

    /**
     * Draw green arrow.
     * @param g graphics
     */
    private static void drawGreenDownArrow(final Graphics2D g)
    {
        drawAmberLamps(g, false);
        Shape shape = SHAPE_CACHE.computeIfAbsent("down_down", (k) ->
        {
            double diagPitchInner = LINE_PITCH / Math.sqrt(2);
            double diagPitchOuter = LINE_PITCH * Math.sqrt(2);
            double line0 = 0.38; // line-start
            double line1 = 0.1; // line-end
            double head = 0.3; // arrow head end points and tip coordinate, together with 0.0 in the other dimension
            Path2D p = new Path2D.Double();
            for (int i = -1; i < 2; i++)
            {
                p.moveTo(i * LINE_PITCH, -line0 * SIZE);
                p.lineTo(i * LINE_PITCH, +line1 * SIZE);
                // from arrow tip to get leds exactly in the arrow tip
                p.moveTo(0.0, head * SIZE - i * diagPitchOuter);
                p.lineTo(-head * SIZE + i * diagPitchInner, -i * diagPitchInner);
                // from arrow tip + one pitch diagonally
                p.moveTo(diagPitchInner, head * SIZE - i * diagPitchOuter - diagPitchInner);
                p.lineTo(head * SIZE - i * diagPitchInner, -i * diagPitchInner);
            }
            return p;
        });
        drawLedStrip(g, shape, LINE_PITCH, GREEN);
    }

    /**
     * Draw end restrictions.
     * @param g graphics
     */
    private static void drawEndRestriction(final Graphics2D g)
    {
        drawAmberLamps(g, false);
        Shape shape = SHAPE_CACHE.computeIfAbsent("end", (k) ->
        {
            Path2D p = new Path2D.Double();
            double r = SIZE * 0.34;
            double diagPitch = END_PITCH / Math.sqrt(2);
            for (int i = -1; i < 2; i++)
            {
                Shape circle = new Ellipse2D.Double(-r + i * END_PITCH, -r + i * END_PITCH, 2 * r - 2 * i * END_PITCH,
                        2 * r - 2 * i * END_PITCH);
                Shape diag = new Line2D.Double(-0.203 * SIZE + i * diagPitch, 0.203 * SIZE + i * diagPitch,
                        0.205 * SIZE + i * diagPitch, -0.205 * SIZE + i * diagPitch);
                p.append(circle, false);
                p.append(diag, false);
            }
            return p;
        });
        drawLedStrip(g, shape, END_PITCH, Color.WHITE);
    }

    /**
     * Draw amber lamps in the corners.
     * @param g graphics
     * @param on whether the amber lights are on
     */
    private static void drawAmberLamps(final Graphics2D g, final boolean on)
    {
        // Corner amber lamps
        double amberPos = 0.38;
        drawAmberLamp(g, -amberPos * SIZE, -amberPos * SIZE, on);
        drawAmberLamp(g, +amberPos * SIZE, -amberPos * SIZE, on);
        drawAmberLamp(g, -amberPos * SIZE, +amberPos * SIZE, on);
        drawAmberLamp(g, +amberPos * SIZE, +amberPos * SIZE, on);
    }

    /**
     * Draw single amber lamp.
     * @param g graphics
     * @param x x-coordinate of center
     * @param y y-coordinate of center
     * @param on whether the amber light is on
     */
    private static void drawAmberLamp(final Graphics2D g, final double x, final double y, final boolean on)
    {
        // main circle
        double rLamp = SIZE * 0.045;
        double rSpeckle = SIZE * 0.006;
        g.setColor(on ? AMBER : EDGE);
        g.fill(new Ellipse2D.Double(x - rLamp, y - rLamp, 2 * rLamp, 2 * rLamp));

        // dotted texture: square grid minus points beyond radius
        g.setColor(on ? AMBER_SPECKLE : EDGE.darker());
        for (int i = -2; i <= 2; i++)
        {
            for (int j = -2; j <= 2; j++)
            {
                double px = x + i * rLamp * 0.35;
                double py = y + j * rLamp * 0.35;
                if (Point2D.distance(px, py, x, y) < rLamp * 0.8)
                {
                    g.fill(new Ellipse2D.Double(px - rSpeckle, py - rSpeckle, 2 * rSpeckle, 2 * rSpeckle));
                }
            }
        }
    }

    /**
     * Draws led strip form shape.
     * @param g graphics
     * @param shape shape
     * @param pitch distance between LEDs along shape
     * @param color led color
     */
    private static void drawLedStrip(final Graphics2D g, final Shape shape, final double pitch, final Color color)
    {
        g.setColor(color);
        for (List<Point2D.Double> line : flattenContour(shape))
        {
            drawLedLine(g, line, pitch);
        }
    }

    /**
     * Iterates the shape and return individual flattened lines.
     * @param shape shape
     * @return lines to be drawn disconnected
     */
    private static Set<List<Point2D.Double>> flattenContour(final Shape shape)
    {
        Set<List<Point2D.Double>> lines = new LinkedHashSet<>();
        List<Point2D.Double> points = new ArrayList<>();
        lines.add(points);
        PathIterator it = shape.getPathIterator(null, FLATNESS);
        double[] c = new double[6];
        while (!it.isDone())
        {
            int type = it.currentSegment(c);
            switch (type)
            {
                case PathIterator.SEG_MOVETO:
                    if (!points.isEmpty())
                    {
                        points = new ArrayList<>();
                        lines.add(points);
                    }
                    points.add(new Point2D.Double(c[0], c[1]));
                    break;
                case PathIterator.SEG_LINETO:
                    points.add(new Point2D.Double(c[0], c[1]));
                    break;
                case PathIterator.SEG_CLOSE:
                    if (!points.isEmpty())
                    {
                        points.add(points.get(0));
                    }
                    break;
                default:
                    throw new IllegalStateException("Unexpected segment type after flattening: " + type);
            }
            it.next();
        }
        return lines;
    }

    /**
     * Draw a single strip of LEDs along a line.
     * @param g graphics
     * @param line line
     * @param pitch distance between LEDs along shape
     */
    private static void drawLedLine(final Graphics2D g, final List<Point2D.Double> line, final double pitch)
    {
        if (line.size() < 2)
        {
            return;
        }
        double totalLength = 0.0;
        for (int i = 1; i < line.size(); i++)
        {
            totalLength += line.get(i - 1).distance(line.get(i));
        }
        int segment = 1;
        double segmentStart = 0.0;
        double segmentEnd = line.get(0).distance(line.get(1));
        for (double target = 0.0; target <= totalLength; target += pitch)
        {
            while (target > segmentEnd && segment < line.size() - 1)
            {
                segmentStart = segmentEnd;
                segment++;
                segmentEnd += line.get(segment - 1).distance(line.get(segment));
            }
            Point2D.Double p0 = line.get(segment - 1);
            Point2D.Double p1 = line.get(segment);
            double f = (target - segmentStart) / (segmentEnd - segmentStart);
            double x = p0.x + f * (p1.x - p0.x);
            double y = p0.y + f * (p1.y - p0.y);
            double ledDiameter = 0.85 * pitch;
            double r = ledDiameter / 2.0;
            g.fill(new Ellipse2D.Double(x - r, y - r, ledDiameter, ledDiameter));
        }
    }

    /**
     * Matrix sign data.
     */
    public interface MatrixSignData extends OtsShape
    {
        /**
         * Returns matrix state.
         * @return matrix state
         */
        MatrixState getState();

        @Override
        default double getZ()
        {
            return DrawLevel.OBJECT.getZ();
        }

        @Override
        default boolean contains(final Point2d point)
        {
            return Math.abs(point.x) < 1.0 && Math.abs(point.y) < 1.0;
        }

    }

    /**
     * Program that draws each sign in to a 512x512 PNG file located in a relative path "matrix/".
     * @param args ignored
     * @throws IOException if a PNG cannot be written
     */
    public static void main(final String[] args) throws IOException
    {
        Files.createDirectories(Paths.get("matrix"));
        double scale = 512.0 / SIZE;
        JvmContext j = new JvmContext("");
        Contextualized c = new Contextualized()
        {
            @Override
            public ContextInterface getContext()
            {
                return j;
            }
        };
        for (MatrixState state : MatrixState.values())
        {
            BufferedImage image = new BufferedImage(512, 512, BufferedImage.TYPE_INT_ARGB);
            Graphics2D graphics = image.createGraphics();
            try
            {
                graphics.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                graphics.translate(256, 256);
                graphics.scale(scale, scale);
                graphics.rotate(-Math.PI / 2.0); // counteract logic that "up" at 0 degrees is to the right of the screen
                MatrixSignAnimation sign = new MatrixSignAnimation(new MatrixSignData()
                {
                    @Override
                    public DirectedPoint2d getLocation()
                    {
                        return null;
                    }

                    @Override
                    public Polygon2d getRelativeContour()
                    {
                        return null;
                    }

                    @Override
                    public MatrixState getState()
                    {
                        return state;
                    }
                }, c);
                sign.paint(graphics, null);
            }
            finally
            {
                graphics.dispose();
            }
            ImageIO.write(image, "png", new File("matrix/" + state.name().toLowerCase() + ".png"));
        }
    }

}
