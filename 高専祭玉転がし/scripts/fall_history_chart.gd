extends Control

const STAGE_COLORS := [Color("#89b4fa"), Color("#cba6f7"), Color("#94e2d5")]
const SECTION_COUNTS := [4, 3, 2]
const AXIS_COLOR := Color("#6c7086")
const GRID_COLOR := Color("#45475a")
const LABEL_COLOR := Color("#a6adc8")

var fall_records: Array[Dictionary] = []


func set_fall_records(records: Array[Dictionary]) -> void:
	fall_records = records.duplicate()
	queue_redraw()


func _draw() -> void:
	var plot := Rect2(40.0, 7.0, size.x - 50.0, size.y - 25.0)
	if plot.size.x <= 0.0 or plot.size.y <= 0.0:
		return
	var font := get_theme_default_font()
	var font_size := 10
	var counts_by_stage := [[0, 0, 0, 0], [0, 0, 0], [0, 0]]
	var total_by_stage := [0, 0, 0]
	var largest_count := 0
	for record in fall_records:
		var stage_index := clampi(int(record.get("stage", 1)) - 1, 0, STAGE_COLORS.size() - 1)
		var section_index := clampi(int(record.get("section", 0)), 0, SECTION_COUNTS[stage_index] - 1)
		counts_by_stage[stage_index][section_index] += 1
		total_by_stage[stage_index] += 1
		largest_count = maxi(largest_count, counts_by_stage[stage_index][section_index])

	var vertical_max := maxi(4, int(ceil(float(largest_count) / 4.0)) * 4)
	for tick in range(4):
		var ratio := float(tick) / 3.0
		var x := plot.position.x + plot.size.x * ratio
		draw_line(Vector2(x, plot.position.y), Vector2(x, plot.end.y), GRID_COLOR, 1.0)
		draw_string(font, Vector2(x + 2.0, size.y - 3.0), "区間%d" % (tick + 1), HORIZONTAL_ALIGNMENT_LEFT, -1.0, font_size, LABEL_COLOR)
	for tick in range(5):
		var ratio := float(tick) / 4.0
		var y := plot.end.y - plot.size.y * ratio
		draw_line(Vector2(plot.position.x, y), Vector2(plot.end.x, y), GRID_COLOR, 1.0)
		draw_string(font, Vector2(1.0, y + 4.0), "%d" % roundi(ratio * vertical_max), HORIZONTAL_ALIGNMENT_LEFT, 34.0, font_size, LABEL_COLOR)
	draw_rect(plot, AXIS_COLOR, false, 1.0)

	for stage_index in range(STAGE_COLORS.size()):
		if total_by_stage[stage_index] == 0:
			continue
		var counts: Array = counts_by_stage[stage_index]
		var color: Color = STAGE_COLORS[stage_index]
		var previous_point := Vector2.ZERO
		for section_index in range(SECTION_COUNTS[stage_index]):
			var point := Vector2(
				plot.position.x + float(section_index) / 3.0 * plot.size.x,
				plot.end.y - float(counts[section_index]) / vertical_max * plot.size.y
			)
			if section_index > 0:
				draw_line(previous_point, point, color, 2.0, true)
			draw_circle(point, 4.0, color)
			draw_string(font, point + Vector2(5.0, -4.0), str(counts[section_index]), HORIZONTAL_ALIGNMENT_LEFT, -1.0, font_size, color)
			previous_point = point
