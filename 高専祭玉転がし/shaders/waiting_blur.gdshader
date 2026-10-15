shader_type canvas_item;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_linear_mipmap;

void fragment() {
	vec3 blurred = textureLod(screen_texture, SCREEN_UV, 1.5).rgb;
	COLOR = vec4(mix(blurred, vec3(0.094, 0.094, 0.145), 0.30), 1.0);
}
