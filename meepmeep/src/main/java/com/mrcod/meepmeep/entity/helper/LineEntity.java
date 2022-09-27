package com.mrcod.meepmeep.entity.helper;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.mrcod.meepmeep.entity.field.BasicThemedEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Stroke;

public class LineEntity extends BasicThemedEntity  {
    private static final Color color = new Color(128, 128, 128, 149);
    private final Vector2d startPosition;
    private final Vector2d endPosition;

    public LineEntity(Vector2d startPosition, Vector2d endPosition, MeepMeep meepMeep) {
        super(meepMeep, "LINE_ENTITY");
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    @Override
    public void render(@NotNull Graphics2D graphics2D, int canvasWidth, int canvasHeight) {
        Vector2d startCoords = FieldUtil.fieldCoordsToScreenCoords(this.startPosition);
        Vector2d endCoords = FieldUtil.fieldCoordsToScreenCoords(this.endPosition);
        BasicStroke stroke = new BasicStroke(4);

        Color oldColor = graphics2D.getColor();
        Stroke oldStroke = graphics2D.getStroke();

        graphics2D.setStroke(stroke);
        graphics2D.setColor(color);
        graphics2D.drawLine((int) (startCoords.getX()), (int) (startCoords.getY()),
                (int) (endCoords.getX()), (int) (endCoords.getY()));
        graphics2D.setStroke(oldStroke);
        graphics2D.setColor(oldColor);
    }

    @Override
    public void setCanvasDimensions(double canvasWidth, double canvasHeight) {
        // NOTHING
    }

    @Override
    public void update(long deltaTime) {
        // NOTHING
    }
}
