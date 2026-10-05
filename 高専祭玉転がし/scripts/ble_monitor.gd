extends Control

const DEVICE_NAME := "KosenBall"
const SENSOR_SERVICE_UUID := "4880c12c-fdcb-4077-8920-a450d7f9b907"
const SENSOR_DATA_CHARACTERISTIC_UUID := "fec26ec4-6d71-4442-9f81-55bc21d658d6"
const MAX_WRITE_BYTES := 20

var bluetooth_manager: Object
var ble_device: Object
var ble_adapter_ready := false
var ble_scanning := false
var connected_to_device := false
var notifications_subscribed := false
var ble_receive_buffer := ""
var notifications_received := 0
var messages_received := 0
var commands_sent := 0
var last_sensor_timestamp := -1
var integrated_yaw := 0.0
var target_model_rotation := Quaternion.IDENTITY

var command_select: OptionButton
var send_button: Button
var status_label: Label
var disconnected_notice: PanelContainer
var stats_label: Label
var telemetry_label: Label
var attitude_label: Label
var log_output: RichTextLabel
var model_root: Node3D


func _ready() -> void:
	_build_ui()
	if not ClassDB.class_exists("BluetoothManager"):
		_set_status("BLE拡張 GDBLE が読み込まれていません。")
		_log("addons/gdble のGDExtensionを確認してください。")
		return

	var manager = ClassDB.instantiate("BluetoothManager")
	if manager == null or not (manager is Node):
		_set_status("BluetoothManagerを作成できませんでした。")
		_log("GDBLE拡張の読み込みに失敗しました。")
		return

	bluetooth_manager = manager
	add_child(manager as Node)
	bluetooth_manager.connect(
		"adapter_initialized", Callable(self, "_on_adapter_initialized")
	)
	bluetooth_manager.connect(
		"device_discovered", Callable(self, "_on_device_discovered")
	)
	bluetooth_manager.connect(
		"error_occurred", Callable(self, "_on_bluetooth_error")
	)
	_set_status("Bluetoothアダプターを初期化しています…")
	_log("GDBLE Bluetooth Low Energy 拡張を初期化しています。")
	bluetooth_manager.call("initialize")


func _process(delta: float) -> void:
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
	title.text = "XIAO MG24 BLE 5.3 Monitor"
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
	notice_label.tooltip_text = "KosenBallのBLE接続を待っています"
	disconnected_notice.add_child(notice_label)

	status_label = Label.new()
	status_label.text = "Bluetoothを準備中…"
	layout.add_child(status_label)

	stats_label = Label.new()
	stats_label.text = "BLE通知 0  |  受信行 0  |  送信コマンド 0"
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
	command_label.text = "Godot → MG24"
	command_row.add_child(command_label)

	command_select = OptionButton.new()
	command_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	command_select.add_item("START")
	command_select.add_item("STOP")
	command_select.add_item("RESET")
	command_select.select(0)
	command_row.add_child(command_select)

	send_button = Button.new()
	send_button.text = "BLEコマンド送信"
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


func _on_adapter_initialized(success: bool, error: String) -> void:
	if not success:
		_set_status("Bluetooth初期化失敗: %s" % error)
		_log("Bluetoothアダプターを初期化できませんでした: %s" % error)
		return

	ble_adapter_ready = true
	_log("Bluetoothアダプターの準備ができました。%s を検索します。" % DEVICE_NAME)
	_start_scan()


func _start_scan() -> void:
	if not ble_adapter_ready or connected_to_device or ble_scanning:
		return

	var operation_id := int(bluetooth_manager.call("start_scan", 0.0))
	if operation_id <= 0:
		_set_status("BLEスキャンを開始できませんでした。")
		_log("GDBLEからスキャン開始要求が受理されませんでした。")
		return

	ble_scanning = true
	_set_status("%s をBLEスキャン中…" % DEVICE_NAME)
	_log("BLEデバイスを検索しています。")


func _on_device_discovered(info: Dictionary) -> void:
	if connected_to_device or ble_device != null:
		return
	if str(info.get("name", "")) != DEVICE_NAME:
		return

	var address := str(info.get("address", ""))
	if address.is_empty():
		_log("KosenBallを検出しましたが、接続アドレスがありません。")
		return

	ble_scanning = false
	bluetooth_manager.call("stop_scan")
	_set_status("KosenBallを検出。接続しています…")
	_log("KosenBallを検出しました (%s)" % address)

	var device = bluetooth_manager.call("get_or_create_device", address)
	if device == null:
		_set_status("BLEデバイスを作成できませんでした。")
		_log("GDBLEからデバイスを取得できませんでした。")
		_restart_scan()
		return

	ble_device = device
	ble_device.connect("connected", Callable(self, "_on_device_connected"))
	ble_device.connect("disconnected", Callable(self, "_on_device_disconnected"))
	ble_device.connect(
		"connection_failed", Callable(self, "_on_connection_failed")
	)
	ble_device.connect(
		"services_discovered", Callable(self, "_on_services_discovered")
	)
	ble_device.connect("ble_event", Callable(self, "_on_device_ble_event"))
	ble_device.connect(
		"operation_failed", Callable(self, "_on_device_operation_failed")
	)
	ble_device.call("connect_async")


func _on_device_connected() -> void:
	_set_connected(true)
	_set_status("BLE接続済み。GATTサービスを検索しています…")
	_log("KosenBallとBLE接続しました。")
	ble_device.call("discover_services")


func _on_services_discovered(services: Array) -> void:
	for service in services:
		if str(service.get("uuid", "")).to_lower() != SENSOR_SERVICE_UUID:
			continue
		for characteristic in service.get("characteristics", []):
			if (
				str(characteristic.get("uuid", "")).to_lower()
				== SENSOR_DATA_CHARACTERISTIC_UUID
				and characteristic.get("properties", {}).get("notify", false)
			):
				var operation_id := int(
					ble_device.call(
						"subscribe_characteristic",
						SENSOR_SERVICE_UUID,
						SENSOR_DATA_CHARACTERISTIC_UUID
					)
				)
				if operation_id <= 0:
					_set_status("センサーデータ通知を購読できませんでした。")
					_log("GATT通知の購読要求が受理されませんでした。")
				else:
					_set_status("GATT通知を購読しています…")
					_log("センサーデータの通知を購読しています。")
				return

	_set_status("必要なGATTサービスまたは通知特性がありません。")
	_log("ファームウェアのService UUIDとCharacteristic UUIDを確認してください。")
	ble_device.call("disconnect")


func _on_device_ble_event(event: Dictionary) -> void:
	var operation := str(event.get("operation", ""))
	var phase := str(event.get("phase", ""))
	var characteristic_uuid := str(
		event.get("characteristic_uuid", "")
	).to_lower()

	if (
		operation == "subscribe"
		and phase == "succeeded"
		and characteristic_uuid == SENSOR_DATA_CHARACTERISTIC_UUID
	):
		notifications_subscribed = true
		send_button.disabled = false
		_set_status("接続済み。BLEセンサーデータを受信中です。")
		_log("BLE通知を購読しました。")
		return

	if (
		operation == "subscribe"
		and phase == "failed"
		and characteristic_uuid == SENSOR_DATA_CHARACTERISTIC_UUID
	):
		_set_status("BLE通知の購読に失敗しました。")
		return

	if (
		operation == "notification"
		and phase == "received"
		and characteristic_uuid == SENSOR_DATA_CHARACTERISTIC_UUID
	):
		var data = event.get("data", PackedByteArray())
		if data is PackedByteArray:
			notifications_received += 1
			_update_stats()
			_ingest_ble_bytes(data)


func _on_device_disconnected() -> void:
	_log("KosenBallとのBLE接続が切れました。再接続します。")
	ble_device = null
	_set_connected(false)
	_restart_scan()


func _on_connection_failed(error: String) -> void:
	_log("KosenBallへのBLE接続に失敗しました: %s" % error)
	ble_device = null
	_set_connected(false)
	_restart_scan()


func _on_device_operation_failed(operation: String, error: String) -> void:
	_log("BLE %s に失敗しました: %s" % [operation, error])


func _on_bluetooth_error(error: String) -> void:
	_log("Bluetoothエラー: %s" % error)


func _restart_scan() -> void:
	await get_tree().create_timer(1.0).timeout
	_start_scan()


func _send_command() -> void:
	if not notifications_subscribed or ble_device == null:
		_log("BLE接続と通知購読が完了してからコマンドを送信してください。")
		return

	var command := command_select.get_item_text(command_select.selected)
	var payload := ("CMD," + command + "\n").to_utf8_buffer()
	if payload.size() > MAX_WRITE_BYTES:
		_log("送信コマンドがBLEの標準パケット長を超えています。")
		return

	var operation_id := int(
		ble_device.call(
			"write_characteristic",
			SENSOR_SERVICE_UUID,
			SENSOR_DATA_CHARACTERISTIC_UUID,
			payload,
			false
		)
	)
	if operation_id <= 0:
		_log("BLEコマンドの送信要求が受理されませんでした。")
		return

	commands_sent += 1
	_update_stats()
	_log("送信コマンド: %s" % command)


func _set_connected(is_connected: bool) -> void:
	var connection_changed := connected_to_device != is_connected
	connected_to_device = is_connected
	disconnected_notice.visible = not is_connected
	notifications_subscribed = false
	send_button.disabled = not (is_connected and notifications_subscribed)

	if not is_connected:
		ble_receive_buffer = ""
		last_sensor_timestamp = -1
		integrated_yaw = 0.0
		target_model_rotation = Quaternion.IDENTITY
		_set_status("未接続。%sを再検索しています…" % DEVICE_NAME)
	elif connection_changed:
		_set_status("BLE接続済み。GATTサービスを検索しています…")


func _set_status(message: String) -> void:
	if is_instance_valid(status_label):
		status_label.text = "状態: " + message


func _ingest_ble_bytes(data: PackedByteArray) -> void:
	ble_receive_buffer += data.get_string_from_utf8()
	if ble_receive_buffer.length() > 1024:
		ble_receive_buffer = ""
		_log("BLE受信データが長すぎるため、未完了の行を破棄しました。")
		return

	while true:
		var newline_index := ble_receive_buffer.find("\n")
		if newline_index < 0:
			break
		var message := ble_receive_buffer.substr(0, newline_index).strip_edges()
		ble_receive_buffer = ble_receive_buffer.substr(newline_index + 1)
		if not message.is_empty():
			_handle_ble_message(message)


func _handle_ble_message(message: String) -> void:
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
		_log("未対応のBLEデータ: %s" % message)


func _update_stats() -> void:
	if is_instance_valid(stats_label):
		stats_label.text = "BLE通知 %d  |  受信行 %d  |  送信コマンド %d" % [
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
	if is_instance_valid(log_output):
		log_output.append_text("%s\n" % message)
