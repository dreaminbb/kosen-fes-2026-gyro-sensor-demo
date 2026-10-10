## PC側の処理:
##   arduino-cli monitorでC6のUSBシリアルを開く -> RX,付きの通知データを改行単位で読む
##   -> DATA行を解析して台を傾け、物理演算で玉を転がす。
## 画面のCMDはUSBシリアルからC6へ渡り、C6がBLE経由でMG24へ転送する。
extends Control

const BallPlatform = preload("res://scripts/ball_platform.gd")
const BAUD_RATE := 115200
const MAX_SERIAL_LINE_BYTES := 16384
const SERIAL_READ_CHUNK_BYTES := 8192
const MAX_SERIAL_READ_BYTES_PER_POLL := 65536
const SENSOR_STALE_USEC := 250000
const MOCHA_BASE := Color("#1e1e2e")
const MOCHA_MANTLE := Color("#181825")
const MOCHA_SURFACE_0 := Color("#313244")
const MOCHA_SURFACE_1 := Color("#45475a")
const MOCHA_SURFACE_2 := Color("#585b70")
const MOCHA_OVERLAY_0 := Color("#6c7086")
const MOCHA_SUBTEXT := Color("#a6adc8")
const MOCHA_TEXT := Color("#cdd6f4")
const MOCHA_LAVENDER := Color("#b4befe")
const MOCHA_BLUE := Color("#89b4fa")
const MOCHA_TEAL := Color("#94e2d5")
const MOCHA_GREEN := Color("#a6e3a1")
const MOCHA_YELLOW := Color("#f9e2af")
const MOCHA_PEACH := Color("#fab387")
const MOCHA_MAUVE := Color("#cba6f7")
const MOCHA_RED := Color("#f38ba8")

var serial_process: Dictionary = {}
var serial_stdio: FileAccess
var serial_stderr: FileAccess
var serial_pid := -1
var connected_to_device := false
var ble_link_ready := false
var serial_receive_buffer := PackedByteArray()
var discarding_serial_line := false
var notifications_received := 0
var messages_received := 0
var commands_sent := 0
var last_sensor_timestamp := -1
var integrated_yaw := 0.0
var sensor_angular_velocity := Vector3.ZERO
var neutral_roll := 0.0
var neutral_pitch := 0.0
var current_sensor_roll := 0.0
var current_sensor_pitch := 0.0
var tilt_calibrated := false
var sensor_pose_available := false
var ball_status_elapsed := 0.0
var latest_sensor_fields := PackedStringArray()
var last_sensor_receive_usec := 0
var sensor_period_ms := 0.0

var command_select: OptionButton
var send_button: Button
var connection_button: Button
var port_input: LineEdit
var status_label: Label
var disconnected_notice: PanelContainer
var stats_label: Label
var telemetry_label: Label
var attitude_label: Label
var log_output: RichTextLabel
var ball_game: BallPlatform
var ball_status_label: Label
var calibrate_button: Button
var connection_panel: PanelContainer
var connection_panel_button: Button
var connection_status_label: Label
var telemetry_detail_label: Label
var control_hint_label: Label


func _ready() -> void:
	# 台の物理更新より先に最新のシリアル入力を取り込む。
	process_physics_priority = -100
	_build_ui()
	port_input.text = _find_default_serial_port()
	if not port_input.text.is_empty():
		_connect_serial()
	else:
		_set_status("C6のシリアルポートを入力してください。")
		_log("ポート例: macOS /dev/cu.usbmodemXXXX、Linux /dev/ttyACM0")
		_toggle_connection_panel(true)
		connection_panel_button.set_pressed_no_signal(true)


func _physics_process(_delta: float) -> void:
	# 120 Hzの物理tickで受信を確認する。姿勢の反映は同じtick内。
	_poll_serial()
	var sensor_fresh := sensor_pose_available and Time.get_ticks_usec() - last_sensor_receive_usec < SENSOR_STALE_USEC
	var focus := get_viewport().gui_get_focus_owner()
	ball_game.keyboard_enabled = not sensor_fresh and not (focus is LineEdit)
	ball_game.set_sensor_angular_velocity(sensor_angular_velocity if sensor_fresh else Vector3.ZERO)
	control_hint_label.text = "BLEコントローラーを傾けて操作" if sensor_fresh else "方向キーで操作  ·  進行方向：画面奥"
	if not sensor_fresh and focus is LineEdit:
		ball_game.set_target_tilt(0.0, 0.0)


func _process(delta: float) -> void:
	ball_status_elapsed += delta
	if ball_status_elapsed >= 0.1 and is_instance_valid(ball_game):
		ball_status_elapsed = 0.0
		ball_status_label.text = ball_game.get_game_status() + "  |  %.2f m/秒" % ball_game.ball.linear_velocity.length()
		_update_stats()
		_refresh_sensor_labels()


func _build_ui() -> void:
	_apply_game_theme()
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	var game_view := SubViewportContainer.new()
	game_view.name = "FullScreenGameView"
	game_view.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	game_view.mouse_filter = Control.MOUSE_FILTER_IGNORE
	game_view.stretch = true
	add_child(game_view)
	var viewport := SubViewport.new()
	viewport.size = Vector2i(1600, 900)
	viewport.own_world_3d = true
	viewport.disable_3d = false
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	game_view.add_child(viewport)
	_create_3d_preview(viewport)

	var overlay := Control.new()
	overlay.name = "GameHUD"
	overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(overlay)

	var top_right := HBoxContainer.new()
	top_right.add_theme_constant_override("separation", 10)
	top_right.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	top_right.position = Vector2(-410, 24)
	top_right.size = Vector2(386, 54)
	overlay.add_child(top_right)
	disconnected_notice = _make_game_panel("ConnectionStatus")
	disconnected_notice.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	disconnected_notice.mouse_filter = Control.MOUSE_FILTER_IGNORE
	top_right.add_child(disconnected_notice)
	connection_status_label = Label.new()
	connection_status_label.text = "●  未接続"
	connection_status_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	connection_status_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	connection_status_label.add_theme_color_override("font_color", MOCHA_RED)
	disconnected_notice.add_child(connection_status_label)
	connection_status_label.text = "●  C6を接続してください"
	connection_panel_button = Button.new()
	connection_panel_button.text = "接続設定"
	connection_panel_button.toggle_mode = true
	connection_panel_button.toggled.connect(_toggle_connection_panel)
	_style_game_button(connection_panel_button, MOCHA_BLUE)
	top_right.add_child(connection_panel_button)

	var game_controls := HBoxContainer.new()
	game_controls.add_theme_constant_override("separation", 8)
	game_controls.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	game_controls.position = Vector2(-482, 88)
	game_controls.size = Vector2(458, 52)
	overlay.add_child(game_controls)
	calibrate_button = Button.new()
	calibrate_button.text = "水平を合わせる"
	calibrate_button.disabled = true
	calibrate_button.pressed.connect(_calibrate_platform)
	_style_game_button(calibrate_button, MOCHA_BLUE)
	game_controls.add_child(calibrate_button)
	var reset_ball_button := Button.new()
	reset_ball_button.text = "もう一度"
	reset_ball_button.pressed.connect(func() -> void: ball_game.reset_ball())
	_style_game_button(reset_ball_button, MOCHA_MAUVE)
	game_controls.add_child(reset_ball_button)
	var overview_button := Button.new()
	overview_button.text = "全体を見る"
	overview_button.toggle_mode = true
	overview_button.toggled.connect(func(enabled: bool) -> void: ball_game.overview_enabled = enabled)
	_style_game_button(overview_button, MOCHA_TEAL)
	game_controls.add_child(overview_button)

	var progress_card := _make_game_panel("RaceStatus")
	_set_panel_rect(progress_card, 0.0, 1.0, 0.0, 1.0, Vector4(24, -116, 600, -24))
	progress_card.custom_minimum_size = Vector2(390, 92)
	overlay.add_child(progress_card)
	var progress_layout := VBoxContainer.new()
	progress_layout.add_theme_constant_override("separation", 4)
	progress_card.add_child(progress_layout)
	var progress_heading := Label.new()
	progress_heading.text = "RACE STATUS"
	progress_heading.add_theme_color_override("font_color", MOCHA_TEAL)
	progress_heading.add_theme_font_size_override("font_size", 12)
	progress_layout.add_child(progress_heading)
	ball_status_label = Label.new()
	ball_status_label.text = "スタート地点に球が落ちています…"
	ball_status_label.add_theme_color_override("font_color", MOCHA_TEXT)
	ball_status_label.add_theme_font_size_override("font_size", 16)
	ball_status_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	progress_layout.add_child(ball_status_label)

	var sensor_card := _make_game_panel("SensorStatus")
	_set_panel_rect(sensor_card, 1.0, 1.0, 1.0, 1.0, Vector4(-544, -148, -24, -24))
	sensor_card.custom_minimum_size = Vector2(520, 124)
	overlay.add_child(sensor_card)
	var sensor_layout := VBoxContainer.new()
	sensor_layout.add_theme_constant_override("separation", 5)
	sensor_card.add_child(sensor_layout)
	var sensor_heading := Label.new()
	sensor_heading.text = "LIVE SENSOR  ·  加速度 g / 角速度 °/秒"
	sensor_heading.add_theme_color_override("font_color", MOCHA_BLUE)
	sensor_heading.add_theme_font_size_override("font_size", 12)
	sensor_layout.add_child(sensor_heading)
	attitude_label = Label.new()
	attitude_label.text = "姿勢  Roll 0°   Pitch 0°   Yaw 0°"
	attitude_label.add_theme_color_override("font_color", MOCHA_LAVENDER)
	sensor_layout.add_child(attitude_label)
	telemetry_label = Label.new()
	telemetry_label.text = "センサーデータを待っています"
	telemetry_label.add_theme_color_override("font_color", MOCHA_SUBTEXT)
	telemetry_label.add_theme_font_size_override("font_size", 12)
	telemetry_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	sensor_layout.add_child(telemetry_label)
	stats_label = Label.new()
	stats_label.text = "受信 0 Hz  ·  最終受信 —"
	stats_label.add_theme_color_override("font_color", MOCHA_GREEN)
	stats_label.add_theme_font_size_override("font_size", 12)
	sensor_layout.add_child(stats_label)
	telemetry_detail_label = Label.new()
	telemetry_detail_label.text = "RX 0 行   ·   MSG 0 行   ·   コマンド 0 件"
	telemetry_detail_label.add_theme_font_size_override("font_size", 11)
	telemetry_detail_label.add_theme_color_override("font_color", MOCHA_OVERLAY_0)
	sensor_layout.add_child(telemetry_detail_label)

	var help_chip := _make_game_panel("ControlHint")
	_set_panel_rect(help_chip, 0.5, 1.0, 0.5, 1.0, Vector4(-238, -64, 238, -20))
	control_hint_label = Label.new()
	control_hint_label.text = "方向キーで操作  ·  進行方向：画面奥"
	control_hint_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	control_hint_label.add_theme_color_override("font_color", MOCHA_YELLOW)
	control_hint_label.add_theme_font_size_override("font_size", 13)
	help_chip.add_child(control_hint_label)
	overlay.add_child(help_chip)

	connection_panel = _make_game_panel("ConnectionSettings")
	_set_panel_rect(connection_panel, 1.0, 0.0, 1.0, 0.0, Vector4(-426, 154, -24, 548))
	connection_panel.custom_minimum_size = Vector2(402, 382)
	connection_panel.visible = false
	overlay.add_child(connection_panel)
	var settings_layout := VBoxContainer.new()
	settings_layout.add_theme_constant_override("separation", 10)
	connection_panel.add_child(settings_layout)
	var settings_heading := Label.new()
	settings_heading.text = "DEVICE & TELEMETRY"
	settings_heading.add_theme_color_override("font_color", MOCHA_LAVENDER)
	settings_heading.add_theme_font_size_override("font_size", 15)
	settings_layout.add_child(settings_heading)
	status_label = Label.new()
	status_label.text = "C6のシリアルポートを選択してください"
	status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	status_label.add_theme_color_override("font_color", MOCHA_SUBTEXT)
	settings_layout.add_child(status_label)
	var port_label := Label.new()
	port_label.text = "C6 SERIAL PORT"
	port_label.add_theme_color_override("font_color", MOCHA_BLUE)
	port_label.add_theme_font_size_override("font_size", 11)
	settings_layout.add_child(port_label)
	var serial_row := HBoxContainer.new()
	serial_row.add_theme_constant_override("separation", 8)
	settings_layout.add_child(serial_row)
	port_input = LineEdit.new()
	port_input.placeholder_text = "/dev/cu.usbmodem…  ·  COM3"
	port_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	serial_row.add_child(port_input)
	connection_button = Button.new()
	connection_button.text = "接続"
	connection_button.pressed.connect(_on_connection_button_pressed)
	serial_row.add_child(connection_button)

	var command_heading := Label.new()
	command_heading.text = "BLE COMMAND"
	command_heading.add_theme_color_override("font_color", MOCHA_PEACH)
	command_heading.add_theme_font_size_override("font_size", 11)
	settings_layout.add_child(command_heading)
	var command_row := HBoxContainer.new()
	command_row.add_theme_constant_override("separation", 8)
	settings_layout.add_child(command_row)
	command_select = OptionButton.new()
	command_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	command_select.add_item("START")
	command_select.add_item("STOP")
	command_select.add_item("RESET")
	command_select.set_item_tooltip(0, "センサーデータの送信を開始")
	command_select.set_item_tooltip(1, "センサーデータの送信を停止")
	command_select.set_item_tooltip(2, "センサーとゲームを再初期化")
	command_select.select(0)
	command_row.add_child(command_select)
	send_button = Button.new()
	send_button.text = "送信"
	send_button.disabled = true
	send_button.pressed.connect(_send_command)
	_style_game_button(send_button, MOCHA_PEACH)
	command_row.add_child(send_button)
	log_output = RichTextLabel.new()
	log_output.bbcode_enabled = false
	log_output.custom_minimum_size.y = 115
	log_output.size_flags_vertical = Control.SIZE_EXPAND_FILL
	log_output.scroll_following = true
	log_output.add_theme_font_size_override("normal_font_size", 12)
	log_output.add_theme_color_override("default_color", MOCHA_SUBTEXT)
	settings_layout.add_child(log_output)

func _set_panel_rect(panel: Control, left: float, top: float, right: float, bottom: float, offsets: Vector4) -> void:
	panel.anchor_left = left
	panel.anchor_top = top
	panel.anchor_right = right
	panel.anchor_bottom = bottom
	panel.offset_left = offsets.x
	panel.offset_top = offsets.y
	panel.offset_right = offsets.z
	panel.offset_bottom = offsets.w


func _make_game_panel(panel_name: String) -> PanelContainer:
	var panel := PanelContainer.new()
	panel.name = panel_name
	var style := StyleBoxFlat.new()
	style.bg_color = Color(MOCHA_MANTLE, 0.94)
	style.border_color = MOCHA_SURFACE_1
	style.set_border_width_all(1)
	style.set_corner_radius_all(12)
	style.content_margin_left = 16.0
	style.content_margin_top = 10.0
	style.content_margin_right = 16.0
	style.content_margin_bottom = 10.0
	style.shadow_color = Color(0.0, 0.0, 0.0, 0.25)
	style.shadow_size = 8
	style.shadow_offset = Vector2(0, 3)
	panel.add_theme_stylebox_override("panel", style)
	return panel


func _style_game_button(button: Button, accent: Color) -> void:
	button.add_theme_font_size_override("font_size", 13)
	for state in ["normal", "hover", "pressed", "focus", "disabled"]:
		var style := StyleBoxFlat.new()
		style.bg_color = MOCHA_SURFACE_0
		style.border_color = MOCHA_SURFACE_1
		style.set_border_width_all(1)
		style.border_color = accent
		style.border_width_bottom = 2
		style.set_corner_radius_all(8)
		style.content_margin_left = 13.0
		style.content_margin_right = 13.0
		style.content_margin_top = 9.0
		style.content_margin_bottom = 9.0
		if state == "hover" or state == "pressed" or state == "focus":
			style.bg_color = accent
		elif state == "disabled":
			style.bg_color = MOCHA_MANTLE
			style.border_color = MOCHA_SURFACE_0
		button.add_theme_stylebox_override(state, style)
	button.add_theme_color_override("font_color", MOCHA_TEXT)
	button.add_theme_color_override("font_hover_color", MOCHA_BASE)
	button.add_theme_color_override("font_pressed_color", MOCHA_BASE)
	button.add_theme_color_override("font_focus_color", MOCHA_BASE)
	button.add_theme_color_override("font_disabled_color", MOCHA_OVERLAY_0)


func _apply_game_theme() -> void:
	var game_theme := Theme.new()
	game_theme.default_font_size = 15
	game_theme.set_color("font_color", "Label", MOCHA_TEXT)
	for control_type in ["Button", "OptionButton", "LineEdit"]:
		game_theme.set_color("font_color", control_type, MOCHA_TEXT)
		game_theme.set_color("font_hover_color", control_type, MOCHA_BASE)
		game_theme.set_color("font_pressed_color", control_type, MOCHA_BASE)
		game_theme.set_color("font_disabled_color", control_type, MOCHA_OVERLAY_0)
		for state in ["normal", "hover", "pressed", "focus", "disabled"]:
			var style := StyleBoxFlat.new()
			style.bg_color = MOCHA_SURFACE_0
			style.border_color = MOCHA_SURFACE_1
			style.set_border_width_all(1)
			style.set_corner_radius_all(8)
			style.content_margin_left = 14.0
			style.content_margin_right = 14.0
			style.content_margin_top = 9.0
			style.content_margin_bottom = 9.0
			if state == "hover" or state == "focus":
				style.bg_color = MOCHA_LAVENDER
				style.border_color = MOCHA_LAVENDER
			elif state == "pressed":
				style.bg_color = MOCHA_BLUE
				style.border_color = MOCHA_BLUE
			elif state == "disabled":
				style.bg_color = MOCHA_MANTLE
				style.border_color = MOCHA_SURFACE_0
			game_theme.set_stylebox(state, control_type, style)
	var line_edit_style := StyleBoxFlat.new()
	line_edit_style.bg_color = MOCHA_BASE
	line_edit_style.border_color = MOCHA_SURFACE_1
	line_edit_style.set_border_width_all(1)
	line_edit_style.set_corner_radius_all(8)
	line_edit_style.content_margin_left = 12.0
	line_edit_style.content_margin_right = 12.0
	game_theme.set_stylebox("normal", "LineEdit", line_edit_style)
	var popup_style := StyleBoxFlat.new()
	popup_style.bg_color = MOCHA_MANTLE
	popup_style.border_color = MOCHA_SURFACE_1
	popup_style.set_border_width_all(1)
	popup_style.set_corner_radius_all(8)
	game_theme.set_stylebox("panel", "PopupMenu", popup_style)
	game_theme.set_color("font_color", "PopupMenu", MOCHA_TEXT)
	theme = game_theme
	RenderingServer.set_default_clear_color(MOCHA_BASE)


func _find_default_serial_port() -> String:
	var entries := DirAccess.get_files_at("/dev")
	var candidates: Array[String] = []
	for entry in entries:
		if entry.begins_with("cu.usbmodem") or entry.begins_with("cu.wchusbserial"):
			candidates.append("/dev/" + entry)
		elif entry.begins_with("ttyACM") or entry.begins_with("ttyUSB"):
			candidates.append("/dev/" + entry)
	candidates.sort()
	return candidates[0] if not candidates.is_empty() else ""


func _on_connection_button_pressed() -> void:
	if connected_to_device:
		_disconnect_serial()
	else:
		_connect_serial()


func _toggle_connection_panel(is_open: bool) -> void:
	connection_panel.visible = is_open
	connection_panel_button.text = "閉じる" if is_open else "接続設定"


func _connect_serial() -> void:
	if connected_to_device:
		return
	var port := port_input.text.strip_edges()
	if port.is_empty():
		_set_status("シリアルポートを入力してください。")
		return
	var cli_path := _find_arduino_cli()
	if cli_path.is_empty():
		_set_status("arduino-cli が見つかりません。")
		_log("arduino-cliをPATHに追加してGodotを起動してください。")
		return

	# C6がBLE通知を出力するシリアルポートをarduino-cli monitor経由で開く。
	var result := OS.execute_with_pipe(
		cli_path,
		["monitor", "--port", port, "--config", "baudrate=%d" % BAUD_RATE],
		false
	)
	if result.is_empty():
		_set_status("シリアルモニターを起動できませんでした。")
		_log("ポートが使用中か、arduino-cli monitorを起動できません。")
		return

	serial_process = result
	serial_stdio = result.get("stdio") as FileAccess
	serial_stderr = result.get("stderr") as FileAccess
	serial_pid = int(result.get("pid", -1))
	serial_receive_buffer.clear()
	discarding_serial_line = false
	_set_connected(true)
	connection_button.text = "切断"
	_style_game_button(connection_button, MOCHA_RED)
	port_input.editable = false
	_set_status("%s を115200 baudで開いています…" % port)
	connection_panel_button.set_pressed_no_signal(false)
	_toggle_connection_panel(false)
	_log("arduino-cli monitorを起動しました: %s" % port)


func _find_arduino_cli() -> String:
	var candidates: Array[String] = []
	if OS.get_name() == "macOS":
		candidates = ["/opt/homebrew/bin/arduino-cli", "/usr/local/bin/arduino-cli"]
	elif OS.get_name() == "Windows":
		candidates = ["arduino-cli.exe"]
	else:
		candidates = ["/usr/bin/arduino-cli", "/usr/local/bin/arduino-cli", "/usr/bin/env"]
	for candidate in candidates:
		if candidate == "/usr/bin/env":
			var lookup_output: Array = []
			if OS.execute(candidate, ["which", "arduino-cli"], lookup_output) == 0 and not lookup_output.is_empty():
				return str(lookup_output[0]).strip_edges()
		elif candidate.get_base_dir() == "." or FileAccess.file_exists(candidate):
			return candidate
	return ""


func _disconnect_serial() -> void:
	if serial_pid > 0 and OS.is_process_running(serial_pid):
		OS.kill(serial_pid)
	if serial_stdio != null:
		serial_stdio.close()
	if serial_stderr != null:
		serial_stderr.close()
	serial_stdio = null
	serial_stderr = null
	serial_process.clear()
	serial_pid = -1
	serial_receive_buffer.clear()
	discarding_serial_line = false
	ble_link_ready = false
	_set_connected(false)
	connection_button.text = "接続"
	_style_game_button(connection_button, MOCHA_GREEN)
	port_input.editable = true
	_toggle_connection_panel(true)
	_log("シリアルモニターを終了しました。")


func _exit_tree() -> void:
	if connected_to_device:
		_disconnect_serial()


func _poll_serial() -> void:
	if not connected_to_device:
		return
	if serial_pid > 0 and not OS.is_process_running(serial_pid):
		_read_pipe(serial_stdio)
		_read_pipe(serial_stderr)
		_log("arduino-cli monitorが終了しました。")
		serial_pid = -1
		serial_stdio = null
		serial_stderr = null
		serial_process.clear()
		_set_connected(false)
		connection_button.text = "接続"
		_style_game_button(connection_button, MOCHA_GREEN)
		port_input.editable = true
		_toggle_connection_panel(true)
		return
	_read_pipe(serial_stdio)
	_read_pipe(serial_stderr)


func _read_pipe(pipe: FileAccess) -> void:
	if pipe == null:
		return
	# 1回の読み取りで止めず、描画を独占しない範囲で溜まったデータを取り込む。
	var remaining := MAX_SERIAL_READ_BYTES_PER_POLL
	while remaining > 0:
		var available := int(pipe.get_length())
		if available <= 0:
			break
		var bytes := pipe.get_buffer(mini(available, mini(remaining, SERIAL_READ_CHUNK_BYTES)))
		if bytes.is_empty():
			break
		remaining -= bytes.size()
		if pipe == serial_stdio:
			_ingest_serial_bytes(bytes)
		else:
			var error_text := bytes.get_string_from_utf8().strip_edges()
			if not error_text.is_empty():
				_log(error_text)


func _ingest_serial_bytes(data: PackedByteArray) -> void:
	# LFまでバイト列を保持し、UTF-8の途中で読み取りが分かれても復元する。
	# 容量制限は1行に適用する。複数の正常な行の合計サイズでは破棄しない。
	var start := 0
	if discarding_serial_line:
		var end := data.find(10)
		if end < 0:
			return
		discarding_serial_line = false
		start = end + 1
	serial_receive_buffer.append_array(data.slice(start))
	var consumed := 0
	while true:
		var newline_index := serial_receive_buffer.find(10, consumed)
		if newline_index < 0:
			break
		if newline_index - consumed > MAX_SERIAL_LINE_BYTES:
			_log("シリアル受信行が長すぎるため破棄しました。")
		else:
			var line := serial_receive_buffer.slice(consumed, newline_index).get_string_from_utf8().strip_edges()
			_handle_serial_line(line)
		consumed = newline_index + 1
	serial_receive_buffer = serial_receive_buffer.slice(consumed)
	if serial_receive_buffer.size() > MAX_SERIAL_LINE_BYTES:
		serial_receive_buffer.clear()
		discarding_serial_line = true
		_log("シリアル受信行が長すぎるため破棄しました。")


func _handle_serial_line(line: String) -> void:
	if line.begins_with("RX,"):
		if not ble_link_ready:
			ble_link_ready = true
			send_button.disabled = false
			connection_status_label.text = "●  BLEデータ受信中"
			connection_status_label.add_theme_color_override("font_color", MOCHA_GREEN)
			_set_status("MG24とBLE接続済み。センサーデータを受信中です。")
			_log("C6からMG24の受信データが届きました。")
		notifications_received += 1
		_handle_serial_message(line.trim_prefix("RX,"))
	elif line.contains("BLE subscribed; bidirectional GATT ready"):
		ble_link_ready = true
		send_button.disabled = false
		connection_status_label.text = "●  BLE接続済み"
		connection_status_label.add_theme_color_override("font_color", MOCHA_GREEN)
		_set_status("MG24とBLE接続済み。センサーデータを受信中です。")
		_log("C6とMG24の双方向BLEリンクが確立しました。")
	elif line.contains("GATT disconnect event") or line.contains("BLE disconnected; will rescan"):
		ble_link_ready = false
		send_button.disabled = true
		connection_status_label.text = "●  BLE再接続中"
		connection_status_label.add_theme_color_override("font_color", MOCHA_YELLOW)
		_set_status("MG24とのBLE切断。C6が再接続しています…")
		_clear_platform_calibration()
	elif line.begins_with("ERROR,") or line.begins_with("WARNING,"):
		_log("C6: " + line)


func _send_command() -> void:
	if not connected_to_device or not ble_link_ready or serial_stdio == null:
		_log("C6のシリアル接続とMG24とのBLEリンク確立後に送信してください。")
		return
	# CMD行をC6の標準入力へ書き、C6側がBLE Write Without ResponseでMG24へ届ける。
	var command := command_select.get_item_text(command_select.selected)
	serial_stdio.store_string("CMD,%s\n" % command)
	serial_stdio.flush()
	commands_sent += 1
	_update_stats()
	_log("C6へコマンド送信: %s" % command)


func _set_connected(is_connected: bool) -> void:
	var connection_changed := connected_to_device != is_connected
	connected_to_device = is_connected
	disconnected_notice.visible = true
	send_button.disabled = not (is_connected and ble_link_ready)
	if is_connected:
		connection_status_label.text = "●  C6接続中 · BLE待機"
		connection_status_label.add_theme_color_override("font_color", MOCHA_YELLOW)
	else:
		connection_status_label.text = "●  C6未接続"
		connection_status_label.add_theme_color_override("font_color", MOCHA_RED)

	if not is_connected:
		ble_link_ready = false
		serial_receive_buffer.clear()
		discarding_serial_line = false
		last_sensor_timestamp = -1
		integrated_yaw = 0.0
		_clear_platform_calibration()
		_set_status("C6のシリアルポート未接続")
		_toggle_connection_panel(true)
		connection_panel_button.set_pressed_no_signal(true)
	elif connection_changed:
		_set_status("C6のシリアルポートに接続しました。BLEリンクを待っています…")


func _set_status(message: String) -> void:
	if is_instance_valid(status_label):
		status_label.text = "状態: " + message


func _handle_serial_message(message: String) -> void:
	if message.begins_with("DATA,"):
		# DATA,timestamp,ax,ay,az,gx,gy,gz の順に受け取り、表示と姿勢推定に使う。
		if not _update_telemetry(message):
			return
		messages_received += 1
	elif message.begins_with("ACK,"):
		var response := message.trim_prefix("ACK,")
		if response == "RESET":
			last_sensor_timestamp = -1
			integrated_yaw = 0.0
			_clear_platform_calibration()
			ball_game.reset_ball()
		_log("MG24からの応答: %s" % response)
		messages_received += 1
	else:
		_log("未対応のC6シリアル行: %s" % message)


func _update_stats() -> void:
	if is_instance_valid(stats_label):
		if sensor_pose_available and sensor_period_ms > 0.0:
			stats_label.text = "受信 %.0f Hz   ·   最終データ %.0f ms前   ·   RX %d" % [
				1000.0 / sensor_period_ms,
				float(Time.get_ticks_usec() - last_sensor_receive_usec) / 1000.0,
				notifications_received
			]
		else:
			stats_label.text = "センサー待機中   ·   RX %d" % notifications_received
		telemetry_detail_label.text = "RX %d 行   ·   MSG %d 行   ·   コマンド %d 件" % [
			notifications_received, messages_received, commands_sent
		]


func _update_telemetry(message: String) -> bool:
	var fields := message.split(",")
	if fields.size() != 8 or not fields[1].is_valid_int():
		_log("形式が不正なセンサーデータを受信しました: %s" % message)
		return false
	for index in range(2, 8):
		if not fields[index].is_valid_float() or not is_finite(fields[index].to_float()):
			_log("数値が不正なセンサーデータを受信しました: %s" % message)
			return false
	latest_sensor_fields = fields
	_update_model_from_sensor(fields)
	return true


func _update_model_from_sensor(fields: PackedStringArray) -> void:
	var sensor_time := fields[1].to_int()
	var accel_x := fields[2].to_float()
	var accel_y := fields[3].to_float()
	var accel_z := fields[4].to_float()
	var gyro_x := fields[5].to_float()
	var gyro_y := fields[6].to_float()
	var gyro_z := fields[7].to_float()
	var gravity_length_squared := accel_x * accel_x + accel_y * accel_y + accel_z * accel_z
	if gravity_length_squared < 0.0001:
		return

	var roll := atan2(accel_y, accel_z)
	var pitch := atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z))
	var elapsed_ms := 0
	if last_sensor_timestamp >= 0:
		elapsed_ms = (sensor_time - last_sensor_timestamp + 4294967296) % 4294967296
	else:
		# 初回は送信側が報告した1周期（現在10 ms）だけ角速度を積分する。
		elapsed_ms = sensor_time
	if elapsed_ms > 0 and elapsed_ms <= 1000:
		integrated_yaw += deg_to_rad(gyro_z) * float(elapsed_ms) / 1000.0
		# 9/10/11 msの時刻丸めでHz表示がちらつかないよう、表示用の周期だけ平滑化する。
		sensor_period_ms = float(elapsed_ms) if sensor_period_ms <= 0.0 else lerpf(sensor_period_ms, float(elapsed_ms), 0.05)
	last_sensor_timestamp = sensor_time
	last_sensor_receive_usec = Time.get_ticks_usec()

	current_sensor_roll = roll
	current_sensor_pitch = pitch
	sensor_angular_velocity = Vector3(deg_to_rad(gyro_x), deg_to_rad(gyro_y), deg_to_rad(gyro_z))
	sensor_pose_available = true
	calibrate_button.disabled = false
	if not tilt_calibrated:
		_calibrate_platform()
	# 現在の取り付け姿勢を水平の基準にし、境界±PIをまたぐ傾きも連続に扱う。
	ball_game.set_target_tilt(
		wrapf(pitch - neutral_pitch, -PI, PI), wrapf(roll - neutral_roll, -PI, PI)
	)


func _refresh_sensor_labels() -> void:
	# 数値表示のみ10 Hz。100 Hzの入力処理・台の更新には待ち時間を入れない。
	if latest_sensor_fields.size() != 8:
		return
	telemetry_label.text = "加速度  %s   %s   %s     角速度  %s   %s   %s" % [
		latest_sensor_fields[2], latest_sensor_fields[3], latest_sensor_fields[4],
		latest_sensor_fields[5], latest_sensor_fields[6], latest_sensor_fields[7]
	]
	attitude_label.text = "姿勢: Roll %.1f°  Pitch %.1f°  Yaw %.1f°" % [
		rad_to_deg(current_sensor_roll), rad_to_deg(current_sensor_pitch), fposmod(rad_to_deg(integrated_yaw), 360.0)
	]


func _calibrate_platform() -> void:
	if not sensor_pose_available:
		return
	neutral_roll = current_sensor_roll
	neutral_pitch = current_sensor_pitch
	tilt_calibrated = true
	ball_game.set_target_tilt(0.0, 0.0)


func _clear_platform_calibration() -> void:
	tilt_calibrated = false
	sensor_pose_available = false
	sensor_angular_velocity = Vector3.ZERO
	last_sensor_receive_usec = 0
	sensor_period_ms = 0.0
	latest_sensor_fields.clear()
	telemetry_label.text = "センサーデータ: 未受信"
	attitude_label.text = "姿勢: 次のセンサーデータを待っています"
	calibrate_button.disabled = true
	if is_instance_valid(ball_game):
		ball_game.set_target_tilt(0.0, 0.0)


func _create_3d_preview(viewport: SubViewport) -> void:
	var world := Node3D.new()
	viewport.add_child(world)

	var environment_node := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = MOCHA_BASE
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = MOCHA_TEXT
	environment.ambient_light_energy = 0.7
	environment_node.environment = environment
	world.add_child(environment_node)

	var camera := Camera3D.new()
	camera.position = Vector3(5.5, 5.7, 6.0)
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.size = 7.2
	camera.keep_aspect = Camera3D.KEEP_HEIGHT
	world.add_child(camera)
	camera.look_at(Vector3(0.0, 0.4, 0.0), Vector3.UP)
	camera.current = true
	# カメラは補間済み球位置を毎描画フレーム参照するため、二重補間を避ける。
	camera.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF

	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-35, -30, 0)
	light.light_energy = 1.5
	light.shadow_enabled = true
	world.add_child(light)

	ball_game = BallPlatform.new()
	world.add_child(ball_game)
	ball_game.follow_camera = camera


func _log(message: String) -> void:
	print(message)
	if is_instance_valid(log_output):
		log_output.append_text("%s\n" % message)
