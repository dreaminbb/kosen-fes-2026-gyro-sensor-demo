extends Control

const DEFAULT_ESP_IP := "192.168.4.1"
const DEFAULT_ESP_PORT := 8888
const HELLO_INTERVAL_MSEC := 1000
const CONNECTION_TIMEOUT_MSEC := 3500

var udp := PacketPeerUDP.new()
var socket_ready := false
var connected_to_esp := false
var next_hello_msec := 0
var last_receive_msec := 0
var packets_sent := 0
var packets_received := 0

var ip_input: LineEdit
var port_input: LineEdit
var message_input: LineEdit
var status_label: Label
var stats_label: Label
var telemetry_label: Label
var log_output: RichTextLabel


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
		var address := "%s:%d" % [udp.get_packet_ip(), udp.get_packet_port()]
		var message := data.get_string_from_utf8()
		packets_received += 1
		last_receive_msec = now
		var was_connected := connected_to_esp
		_set_connected(true)
		stats_label.text = "送信 %d  |  受信 %d  |  最終受信元 %s" % [packets_sent, packets_received, address]

		if message == "HELLO_ACK":
			if not was_connected:
				_log("ESP32との接続を確認しました (%s)" % address)
		elif message.begins_with("DATA,"):
			_update_telemetry(message)
		elif message.begins_with("ACK,"):
			_log("ESP32からの応答: %s" % message.trim_prefix("ACK,"))
		else:
			_log("受信: %s (%s)" % [message, address])

	if connected_to_esp and now - last_receive_msec > CONNECTION_TIMEOUT_MSEC:
		_set_connected(false)
		_log("ESP32からの受信が途絶えました。Wi-Fiとファームウェアを確認してください。")


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

	var title := Label.new()
	title.text = "ESP32 UDP Monitor"
	title.add_theme_font_size_override("font_size", 24)
	layout.add_child(title)

	status_label = Label.new()
	status_label.text = "UDPソケットを準備中…"
	layout.add_child(status_label)

	stats_label = Label.new()
	stats_label.text = "送信 0  |  受信 0"
	layout.add_child(stats_label)

	telemetry_label = Label.new()
	telemetry_label.text = "センサーデータ: 未受信"
	layout.add_child(telemetry_label)

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
	if connected_to_esp == is_connected:
		return
	connected_to_esp = is_connected
	if connected_to_esp:
		_set_status("接続中: ESP32とUDP通信しています")
	else:
		_set_status("未接続: ESP32を検索中…")


func _set_status(message: String) -> void:
	if is_instance_valid(status_label):
		status_label.text = "状態: " + message


func _update_telemetry(message: String) -> void:
	var fields := message.split(",")
	if fields.size() != 8:
		_log("形式が不正なセンサーデータを受信しました: %s" % message)
		return
	telemetry_label.text = "センサー (仮データ)  ax=%s ay=%s az=%s  gx=%s gy=%s gz=%s" % [
		fields[2], fields[3], fields[4], fields[5], fields[6], fields[7]
	]


func _log(message: String) -> void:
	if is_instance_valid(log_output):
		log_output.append_text("%s\n" % message)
