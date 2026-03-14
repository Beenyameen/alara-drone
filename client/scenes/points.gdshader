shader_type spatial;
render_mode unshaded;

uniform float ceiling_y = 3.0;

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	if (world_pos.y > ceiling_y) {
		discard;
	}
	ALBEDO = COLOR.rgb;
}
