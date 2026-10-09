## 処理の流れ: arduino-cli monitorを起動し、C6のUSBシリアル出力を読む。
## DATAで3D姿勢を更新し、画面のコマンドをシリアル経由でMG24へ送る。
extends Control

const BAUD_RATE := 115200
const MAX_SERIAL_LINE_LENGTH := 1024

var serial_process: Dictionary = {}
var serial_stdio: FileAccess
var serial_stderr: FileAccess
var serial_pid := -1
var connected_to_device := false
var ble_link_ready := false
var serial_receive_buffer := ""
var notifications_received := 0
var messages_received := 0
var commands_sent := 0
var last_sensor_timestamp := -1
var integrated_yaw := 0.0
var target_model_rotation := Quaternion.IDENTITY

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
var model_root: Node3D


func _ready() -> void:
	_build_ui()
	port_input.text = _find_default_serial_port()
	if not port_input.text.is_empty():
		_connect_serial()
	else:
		_set_status("C6のシリアルポートを入力してください。")
		_log("ポート例: macOS /dev/cu.usbmodemXXXX、Linux /dev/ttyACM0")


func _process(delta: float) -> void:
	_poll_serial()
	if is_instance_valid(model_root):
		model_root.quaternion = model_root.quaternion.slerp(
			target_model_rotation, 1.0 - exp(-8.0 * delta)
		)


func _build_ui() -> void:
	var margin := MarginContainer.new()
	margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", 24)
	margin.add_theme_constant_override("margin_top", 24)
	margin.add_theme_constant_override("margin_right", 24)
	margin.add_theme_constant_override("margin_bottom", 24)
	add_child(margin)

	var layout := VBoxContainer.new()
	layout.add_theme_constant_override("separation", 12)
	margin.add_child(layout)

	var header := HBoxContainer.new()
	header.custom_minimum_size.y = 40
	header.add_theme_constant_override("separation", 16)
	layout.add_child(header)

	var title := Label.new()
	title.text = "XIAO MG24 ↔ ESP32-C6 BLE Monitor"
	title.add_theme_font_size_override("font_size", 24)
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(title)

	disconnected_notice = PanelContainer.new()
	disconnected_notice.name = "DisconnectedNotice"
	disconnected_notice.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	disconnected_notice.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var notice_style := StyleBoxFlat.new()
	notice_style.bg_color = Color("7f1d1d")
	notice_style.set_corner_radius_all(8)
	notice_style.content_margin_left = 16
	notice_style.content_margin_right = 16
	notice_style.content_margin_top = 8
	notice_style.content_margin_bottom = 8
	disconnected_notice.add_theme_stylebox_override("panel", notice_style)
	header.add_child(disconnected_notice)
	var notice_label := Label.new()
	notice_label.text = "未接続"
	notice_label.add_theme_color_override("font_color", Color("fee2e2"))
	notice_label.tooltip_text = "C6のUSBシリアル接続を待っています"
	disconnected_notice.add_child(notice_label)

	status_label = Label.new()
	status_label.text = "C6シリアルを準備中…"
	layout.add_child(status_label)

	var serial_row := HBoxContainer.new()
	serial_row.add_theme_constant_override("separation", 8)
	layout.add_child(serial_row)
	var port_label := Label.new()
	port_label.text = "C6ポート"
	serial_row.add_child(port_label)
	port_input = LineEdit.new()
	port_input.placeholder_text = "/dev/cu.usbmodemXXXX または COM3"
	port_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	serial_row.add_child(port_input)
	connection_button = Button.new()
	connection_button.text = "シリアル接続"
	connection_button.pressed.connect(_on_connection_button_pressed)
	serial_row.add_child(connection_button)

	stats_label = Label.new()
	stats_label.text = "シリアル受信 0  |  受信行 0  |  送信コマンド 0"
	layout.add_child(stats_label)

	telemetry_label = Label.new()
	telemetry_label.text = "センサーデータ: 未受信"
	layout.add_child(telemetry_label)

	attitude_label = Label.new()
	attitude_label.text = "姿勢: Roll 0°  Pitch 0°  Yaw 0°"
	layout.add_child(attitude_label)

	var preview_container := SubViewportContainer.new()
	preview_container.custom_minimum_size = Vector2(480, 300)
	preview_container.size_flags_stretch_ratio = 3.0
	preview_container.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	preview_container.size_flags_vertical = Control.SIZE_EXPAND_FILL
	preview_container.stretch = true
	layout.add_child(preview_container)

	var viewport := SubViewport.new()
	viewport.size = Vector2i(960, 600)
	viewport.own_world_3d = true
	viewport.disable_3d = false
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	preview_container.add_child(viewport)
	_create_3d_preview(viewport)

	var command_row := HBoxContainer.new()
	command_row.add_theme_constant_override("separation", 8)
	layout.add_child(command_row)

	var command_label := Label.new()
	command_label.text = "Godot → C6 → MG24"
	command_row.add_child(command_label)

	command_select = OptionButton.new()
	command_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	command_select.add_item("START")
	command_select.add_item("STOP")
	command_select.add_item("RESET")
	command_select.select(0)
	command_row.add_child(command_select)

	send_button = Button.new()
	send_button.text = "コマンド送信"
	send_button.disabled = true
	send_button.pressed.connect(_send_command)
	command_row.add_child(send_button)

	log_output = RichTextLabel.new()
	log_output.bbcode_enabled = false
	log_output.custom_minimum_size.y = 60
	log_output.scroll_following = true
	log_output.size_flags_vertical = Control.SIZE_EXPAND_FILL
	log_output.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	layout.add_child(log_output)


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
	serial_receive_buffer = ""
	_set_connected(true)
	connection_button.text = "切断"
	port_input.editable = false
	_set_status("%s を115200 baudで開いています…" % port)
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
	serial_receive_buffer = ""
	ble_link_ready = false
	_set_connected(false)
	connection_button.text = "シリアル接続"
	port_input.editable = true
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
		connection_button.text = "シリアル接続"
		port_input.editable = true
		return
	_read_pipe(serial_stdio)
	_read_pipe(serial_stderr)


func _read_pipe(pipe: FileAccess) -> void:
	if pipe == null:
		return
	var available := int(pipe.get_length())
	if available <= 0:
		return
	var bytes := pipe.get_buffer(mini(available, 8192))
	if bytes.is_empty():
		return
	if pipe == serial_stdio:
		_ingest_serial_bytes(bytes)
	else:
		var error_text := bytes.get_string_from_utf8().strip_edges()
		if not error_text.is_empty():
			_log(error_text)


func _ingest_serial_bytes(data: PackedByteArray) -> void:
	serial_receive_buffer += data.get_string_from_utf8()
	if serial_receive_buffer.length() > MAX_SERIAL_LINE_LENGTH:
		serial_receive_buffer = ""
		_log("シリアル受信行が長すぎるため破棄しました。")
		return

	while true:
		var newline_index := serial_receive_buffer.find("\n")
		if newline_index < 0:
			break
		var line := serial_receive_buffer.substr(0, newline_index).strip_edges()
		serial_receive_buffer = serial_receive_buffer.substr(newline_index + 1)
		if line.begins_with("RX,"):
			if not ble_link_ready:
				ble_link_ready = true
				send_button.disabled = false
				_set_status("MG24とBLE接続済み。センサーデータを受信中です。")
				_log("C6からMG24の受信データが届きました。")
			notifications_received += 1
			_update_stats()
			_handle_serial_message(line.trim_prefix("RX,"))
		elif line.contains("BLE subscribed; bidirectional GATT ready"):
			ble_link_ready = true
			send_button.disabled = false
			_set_status("MG24とBLE接続済み。センサーデータを受信中です。")
			_log("C6とMG24の双方向BLEリンクが確立しました。")
		elif line.contains("GATT disconnect event") or line.contains("BLE disconnected; will rescan"):
			ble_link_ready = false
			send_button.disabled = true
			_set_status("MG24とのBLE切断。C6が再接続しています…")
		elif line.begins_with("ERROR,") or line.begins_with("WARNING,"):
			_log("C6: " + line)


func _send_command() -> void:
	if not connected_to_device or not ble_link_ready or serial_stdio == null:
		_log("C6のシリアル接続とMG24とのBLEリンク確立後に送信してください。")
		return
	var command := command_select.get_item_text(command_select.selected)
	serial_stdio.store_string("CMD,%s\n" % command)
	serial_stdio.flush()
	commands_sent += 1
	_update_stats()
	_log("C6へコマンド送信: %s" % command)


func _set_connected(is_connected: bool) -> void:
	var connection_changed := connected_to_device != is_connected
	connected_to_device = is_connected
	disconnected_notice.visible = not is_connected
	send_button.disabled = not (is_connected and ble_link_ready)

	if not is_connected:
		ble_link_ready = false
		serial_receive_buffer = ""
		last_sensor_timestamp = -1
		integrated_yaw = 0.0
		target_model_rotation = Quaternion.IDENTITY
		_set_status("C6のシリアルポート未接続")
	elif connection_changed:
		_set_status("C6のシリアルポートに接続しました。BLEリンクを待っています…")


func _set_status(message: String) -> void:
	if is_instance_valid(status_label):
		status_label.text = "状態: " + message


func _handle_serial_message(message: String) -> void:
	if message.begins_with("DATA,"):
		if not _update_telemetry(message):
			return
		messages_received += 1
		_update_stats()
	elif message.begins_with("ACK,"):
		var response := message.trim_prefix("ACK,")
		if response == "RESET":
			last_sensor_timestamp = -1
			integrated_yaw = 0.0
			target_model_rotation = Quaternion.IDENTITY
		_log("MG24からの応答: %s" % response)
		messages_received += 1
		_update_stats()
	else:
		_log("未対応のC6シリアル行: %s" % message)


func _update_stats() -> void:
	if is_instance_valid(stats_label):
		stats_label.text = "シリアル行 %d  |  受信行 %d  |  送信コマンド %d" % [
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
	telemetry_label.text = "センサー (仮データ)  ax=%s ay=%s az=%s  gx=%s gy=%s gz=%s" % [
		fields[2], fields[3], fields[4], fields[5], fields[6], fields[7]
	]
	_update_model_from_sensor(fields)
	return true


func _update_model_from_sensor(fields: PackedStringArray) -> void:
	var sensor_time := fields[1].to_int()
	var accel_x := fields[2].to_float()
	var accel_y := fields[3].to_float()
	var accel_z := fields[4].to_float()
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
		# The first sample describes the first 100 ms interval of motion.
		elapsed_ms = sensor_time
	if elapsed_ms > 0 and elapsed_ms <= 1000:
		integrated_yaw += deg_to_rad(gyro_z) * float(elapsed_ms) / 1000.0
	last_sensor_timestamp = sensor_time

	target_model_rotation = Basis.from_euler(Vector3(pitch, integrated_yaw, roll)).get_rotation_quaternion()
	attitude_label.text = "姿勢: Roll %.1f°  Pitch %.1f°  Yaw %.1f°" % [
		rad_to_deg(roll), rad_to_deg(pitch), fposmod(rad_to_deg(integrated_yaw), 360.0)
	]


func _create_3d_preview(viewport: SubViewport) -> void:
	var world := Node3D.new()
	viewport.add_child(world)

	var environment_node := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color("111827")
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color("cbd5e1")
	environment.ambient_light_energy = 0.7
	environment_node.environment = environment
	world.add_child(environment_node)

	var camera := Camera3D.new()
	camera.position = Vector3(2.8, 2.4, 3.6)
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.size = 3.4
	camera.keep_aspect = Camera3D.KEEP_HEIGHT
	world.add_child(camera)
	camera.look_at(Vector3.ZERO, Vector3.UP)
	camera.current = true

	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-35, -30, 0)
	light.light_energy = 1.5
	world.add_child(light)

	model_root = Node3D.new()
	world.add_child(model_root)
	_add_box(model_root, Vector3(0.7, 0.9, 0.48), Vector3.ZERO, Color("38bdf8"))
	_add_box(model_root, Vector3(1.4, 0.055, 0.055), Vector3(0.72, 0.0, 0.0), Color("fb7185"))
	_add_box(model_root, Vector3(0.055, 1.35, 0.055), Vector3(0.0, 0.68, 0.0), Color("4ade80"))
	_add_box(model_root, Vector3(0.055, 0.055, 1.4), Vector3(0.0, 0.0, 0.72), Color("facc15"))
	_add_cone(model_root, Vector3(0.0, 0.0, -0.38), Color("f97316"))


func _add_box(parent: Node3D, size: Vector3, position: Vector3, color: Color) -> void:
	var mesh_instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mesh_instance.mesh = mesh
	mesh_instance.position = position
	mesh_instance.material_override = _make_material(color)
	parent.add_child(mesh_instance)


func _add_cone(parent: Node3D, position: Vector3, color: Color) -> void:
	var mesh_instance := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.top_radius = 0.0
	mesh.bottom_radius = 0.17
	mesh.height = 0.34
	mesh_instance.mesh = mesh
	mesh_instance.position = position
	mesh_instance.rotation.x = -PI / 2.0
	mesh_instance.material_override = _make_material(color)
	parent.add_child(mesh_instance)


func _make_material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.metallic = 0.18
	material.roughness = 0.32
	return material


func _log(message: String) -> void:
	print(message)
	if is_instance_valid(log_output):
		log_output.append_text("%s\n" % message)
