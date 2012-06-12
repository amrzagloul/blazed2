module blaze.collision.shapes.bzShapeType;

// Note: not put in shape.d due to 'forward reference' errors in contactFactory

/// The various collision shape types supported by Blaze.
enum bzShapeType
{
	UNKNOWN = -1,
	CIRCLE,
	POLYGON,
    EDGE,
	FLUID,
	SHAPE_COUNT
}
