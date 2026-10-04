extends Control

const DEFAULT_ESP_IP := "192.168.4.1"
const DEFAULT_ESP_PORT := 8888
const HELLO_INTERVAL_MSEC := 1000
const CONNECTION_TIMEOUT_MSEC := 3500

var udp := PacketPeerUDP.new()
var socket_ready := false
var active_esp_ip := ""
var active_esp_port := 0
var connected_to_esp := false
var next_hello_msec := 0
var last_receive_msec := 0
var packets_sent := 0
var packets_received := 0
var last_sensor_timestamp := -1
var integrated_yaw := 0.0
var target_model_rotation := Quaternion.IDENTITY

var ip_input: LineEdit
var port_input: LineEdit
var message_input: LineEdit
var status_label: Label
var disconnected_notice: PanelContainer
var stats_label: Label
var telemetry_label: Label
var attitude_label: Label
var log_output: RichTextLabel
var model_root: Node3D


func _ready() -> void:
	_build_ui()
	var error := udp.bind(0)
	if error != OK:
		_set_status("UDPソケットを開けられません: %s" % error_string(error))
		_log("GodotでUDP受信用ソケットを作れませんでした。")
	else:
		socket_ready = true
		_set_status("UDPソケット準備完了。ESP32を検索中…")
		_log("受信用UDPポート %d を開きました。" % udp.get_local_port())
		next_hello_msec = Time.get_ticks_msec()


func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec()
	if socket_ready and now >= next_hello_msec:
		_send_packet("HELLO", false)
		next_hello_msec = now + HELLO_INTERVAL_MSEC

	while udp.get_available_packet_count() > 0:
		var data := udp.get_packet()
		if udp.get_packet_ip() != active_esp_ip or udp.get_packet_port() != active_esp_port:
			continue
		var address := "%s:%d" % [udp.get_packet_ip(), udp.get_packet_port()]
		var message := data.get_string_from_utf8()
		packets_received += 1
		stats_label.text = "送信 %d  |  受信 %d  |  最終受信元 %s" % [packets_sent, packets_received, address]

		# UDP has no connection handshake; only valid ESP responses prove it is reachable.
		if message == "HELLO_ACK":
			pass
		elif message.begins_with("DATA,"):
			if not _update_telemetry(message):
				continue
		elif message.begins_with("ACK,"):
			_log("ESP32からの応答: %s" % message.trim_prefix("ACK,"))
		else:
			_log("未対応の受信データ: %s (%s)" % [message, address])
			continue

		if not connected_to_esp:
			_log("ESP32との接続を確認しました (%s)" % address)
		last_receive_msec = now
		_set_connected(true)

	if connected_to_esp and now - last_receive_msec > CONNECTION_TIMEOUT_MSEC:
		_set_connected(false)
		_log("ESP32からの受信が途絶えました。Wi-Fiとファームウェアを確認してください。")

	if is_instance_valid(model_root):
		model_root.quaternion = model_root.quaternion.slerp(
			target_model_rotation, 1.0 - exp(-8.0 * _delta)
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
	title.text = "ESP32 UDP Monitor"
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
	notice_label.tooltip_text = "ESP32からのUDP応答を待っています"
	disconnected_notice.add_child(notice_label)

	status_label = Label.new()
	status_label.text = "UDPソケットを準備中…"
	layout.add_child(status_label)

	stats_label = Label.new()
	stats_label.text = "送信 0  |  受信 0"
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
	# Keep the preview independent of the main scene's cameras and environments.
	viewport.own_world_3d = true
	viewport.disable_3d = false
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	preview_container.add_child(viewport)
	_create_3d_preview(viewport)

	var destination_row := HBoxContainer.new()
	destination_row.add_theme_constant_override("separation", 8)
	layout.add_child(destination_row)

	var ip_label := Label.new()
	ip_label.text = "ESP32 IP"
	destination_row.add_child(ip_label)

	ip_input = LineEdit.new()
	ip_input.text = DEFAULT_ESP_IP
	ip_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	destination_row.add_child(ip_input)

	var port_label := Label.new()
	port_label.text = "Port"
	destination_row.add_child(port_label)

	port_input = LineEdit.new()
	port_input.text = str(DEFAULT_ESP_PORT)
	port_input.custom_minimum_size.x = 100
	port_input.max_length = 5
	destination_row.add_child(port_input)

	message_input = LineEdit.new()
	message_input.placeholder_text = "送信するメッセージを入力"
	message_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	message_input.text_submitted.connect(_on_message_submitted)
	layout.add_child(message_input)

	var send_button := Button.new()
	send_button.text = "UDP送信"
	send_button.pressed.connect(_send_message)
	layout.add_child(send_button)

	log_output = RichTextLabel.new()
	log_output.bbcode_enabled = false
	log_output.custom_minimum_size.y = 60
	log_output.scroll_following = true
	log_output.size_flags_vertical = Control.SIZE_EXPAND_FILL
	log_output.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	layout.add_child(log_output)


func _on_message_submitted(_text: String) -> void:
	_send_message()


func _send_message() -> void:
	if message_input.text.is_empty():
		_log("送信メッセージを入力してください。")
		return
	_send_packet("MSG," + message_input.text, true)


func _send_packet(message: String, report_errors: bool) -> void:
	if not socket_ready:
		return
	var target_ip := ip_input.text.strip_edges()
	var target_port := port_input.text.to_int()
	if target_ip.is_empty() or target_port < 1 or target_port > 65535:
		if report_errors:
			_log("IPアドレスまたはポート番号を確認してください。")
		return

	if target_ip != active_esp_ip or target_port != active_esp_port:
		active_esp_ip = target_ip
		active_esp_port = target_port
		_set_connected(false)
		last_sensor_timestamp = -1

	var error := udp.set_dest_address(target_ip, target_port)
	if error != OK:
		if report_errors:
			_log("送信先を設定できませんでした: %s" % error_string(error))
		return

	error = udp.put_packet(message.to_utf8_buffer())
	if error != OK:
		if report_errors:
			_log("UDP送信に失敗しました: %s" % error_string(error))
		return

	packets_sent += 1
	stats_label.text = "送信 %d  |  受信 %d" % [packets_sent, packets_received]
	if report_errors:
		_log("送信: %s → %s:%d" % [message, target_ip, target_port])


func _set_connected(is_connected: bool) -> void:
	disconnected_notice.visible = not is_connected
	if connected_to_esp == is_connected:
		return
	connected_to_esp = is_connected
	if connected_to_esp:
		_set_status("接続中: ESP32とUDP通信しています")
	else:
		last_sensor_timestamp = -1
		_set_status("未接続: ESP32を検索中…")


func _set_status(message: String) -> void:
	if is_instance_valid(status_label):
		status_label.text = "状態: " + message


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

	# Accelerometer gravity gives pitch/roll. It cannot measure yaw, so integrate
	# the Z gyro for turns around the vertical axis (including a full 360° turn).
	var roll := atan2(accel_y, accel_z)
	var pitch := atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z))
	if last_sensor_timestamp >= 0:
		var elapsed_ms := (sensor_time - last_sensor_timestamp + 4294967296) % 4294967296
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
	# Orthographic framing keeps the model large enough to see in a wide UI panel.
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
	if is_instance_valid(log_output):
		log_output.append_text("%s\n" % message)
