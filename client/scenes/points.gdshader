shader_type spatial;
render_mode unshaded;

uniform float ceiling_y = 3.0;

varying float point_y;

void vertex() {
	// MODEL_MATRIX[3].y gives the global Y position of the instance. 
	// Since WorldContent is shifted so the lowest point is at Global Y = 0,
	// this perfectly matches our UI bounds of 0 to (rawMaxY - rawMinY)
	point_y = MODEL_MATRIX[3].y;
}

void fragment() {
	if (point_y > ceiling_y) {
		discard;
	}
	ALBEDO = COLOR.rgb;
}
