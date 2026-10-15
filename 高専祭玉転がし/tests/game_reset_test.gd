# 実機なしでゲーム状態・受信プロトコル・固定/落下を確認する。
# godot --headless --path 高専祭玉転がし --script res://tests/game_reset_test.gd
extends SceneTree

class MonitorFixture:
	extends "res://scripts/ble_monitor.gd"
	var sent: Array[String] = []
	var usb_available := true

	func _ready() -> void:
		_build_ui()
		_update_waiting_overlay()
		set_physics_process(false)

	func _write_bridge_command(command: String) -> bool:
		if not usb_available:
			return false
		sent.append(command)
		return true


var failures := 0


func _initialize() -> void:
	_run.call_deferred()


func check(condition: bool, description: String) -> void:
	if not condition:
		failures += 1
		push_error(description)


func _run() -> void:
	var monitor := MonitorFixture.new()
	root.add_child(monitor)
	var game := monitor.ball_game
	game.keyboard_enabled = false
	check(game.ball.freeze, "起動直後は玉を空中に固定する")
	check(monitor.waiting_title.text == "ボタンを押してスタート", "初期案内")
	monitor._handle_serial_line("RX,GAMERESET")
	check(monitor.show_resume_prompt and game.ball.freeze, "初回押下は準備だけを行う")
	check(monitor.waiting_title.text == "もう一度押して再開", "準備後の案内")
	var held_position := game.ball.global_position
	for frame in range(12):
		await physics_frame
	check(game.ball.global_position.is_equal_approx(held_position), "待機中に玉が落下しない")
	check(game.elapsed_seconds == 0.0 and game.fall_count == 0, "待機中は計時・落下判定をしない")
	monitor._update_game_state(179.0)
	check(monitor.sent.is_empty(), "180秒未満はSTOPしない")
	monitor._update_game_state(1.0)
	check(monitor.sent == ["STOP"], "180秒でSTOPを送信する")
	monitor._handle_serial_line("GAME,SLEEPING")
	monitor._update_game_state(10.0)
	check(monitor.sent == ["STOP"], "休止確認後はSTOPを再送しない")
	monitor._handle_serial_line("GAME,BUTTON_PENDING")
	monitor._update_game_state(0.0)
	check(monitor.sent == ["STOP"] and monitor.game_state == monitor.GameState.WAITING, "接続状況通知だけではボタン操作を実行しない")
	check(monitor.button_link_pending, "C6のGATT接続中の状況を表示する")
	monitor._handle_serial_line("GAME,WAKING")
	monitor._update_game_state(0.0)
	check(monitor.sent == ["STOP"], "START要求の状況通知も押下として扱わない")
	monitor._handle_serial_line("RX,GAMERESET")
	monitor._update_game_state(0.0)
	check(monitor.sent == ["STOP", "START"], "C6のGAMERESET受信後にSTARTを要求する")
	monitor._handle_serial_line("RX,GAMERESET")
	check(game.ball.freeze and monitor.sent == ["STOP", "START"], "復帰中の追加押下で二重に開始しない")
	monitor._handle_serial_line("RX,ACK,START")
	monitor._handle_serial_line("RX,DATA,10000,0,0,1,0,0,0")
	monitor._update_game_state(0.0)
	check(game.ball.freeze, "ACKとDATAだけではC6のREADY確認前に開始しない")
	monitor._handle_serial_line("GAME,READY")
	monitor._update_game_state(0.0)
	check(not game.ball.freeze and game.race_started, "READYと新しいDATAの確認後に玉を落とす")
	check(not monitor.waiting_overlay.visible, "開始後はぼかしと案内を消す")
	for frame in range(20):
		await physics_frame
	check(game.ball.global_position.y < held_position.y, "開始後は物理演算で落下する")
	check(game.elapsed_seconds > 0.0, "開始後は計時する")
	game.checkpoint_index = 2
	game.fall_count = 4
	monitor._handle_serial_line("GAMERESET")
	check(game.ball.freeze and game.checkpoint_index == 0 and game.fall_count == 0, "再押下で全記録をリセットして固定する")
	check(game.ball.global_position.is_equal_approx(held_position), "チェックポイントではなくSTART上空へ戻す")
	monitor._update_game_state(179.0)
	check(monitor.sent.size() == 2, "再リセットから180秒を計り直す")
	monitor.usb_available = false
	monitor._on_game_button_pressed()
	monitor._update_game_state(0.0)
	check(not monitor.wake_command_sent and game.ball.freeze, "USB切断中は固定したまま待つ")
	monitor.usb_available = true
	monitor._update_game_state(0.0)
	monitor._handle_serial_line("GAME,READY")
	monitor._update_game_state(0.0)
	check(game.ball.freeze, "以前のDATAを使って復帰しない")
	monitor._handle_serial_line("RX,DATA,20000,0,0,0,0,0,0")
	monitor._update_game_state(0.0)
	check(game.ball.freeze, "重力方向が不明なDATAでは復帰しない")
	monitor._handle_serial_line("RX,DATA,30000,0,0,1,0,0,0")
	monitor._update_game_state(0.0)
	check(not game.ball.freeze, "USB復帰後も開始できる")
	monitor._handle_serial_line("RX,ACK,RESET")
	check(game.ball.freeze and game.elapsed_seconds == 0.0, "手動RESET応答も待機へ戻す")
	var sent_before_resume := monitor.sent.size()
	monitor._on_game_button_pressed()
	monitor._update_game_state(181.0)
	check(monitor.sent.slice(sent_before_resume) == ["START"], "停止期限前の再開はSTOPタイマーを取り消す")
	monitor._handle_serial_line("GAME,READY")
	monitor._handle_serial_line("BLE disconnected; will rescan")
	monitor._handle_serial_line("RX,DATA,40000,0,0,1,0,0,0")
	monitor._update_game_state(0.0)
	check(game.ball.freeze, "復帰途中の切断では以前のREADYを破棄する")
	monitor._handle_serial_line("GAME,READY")
	monitor._update_game_state(0.0)
	check(not game.ball.freeze, "再接続後の新しいREADYで開始する")
	monitor._enter_waiting()
	monitor.mg24_stop_delay_seconds = 0.0
	monitor._update_game_state(0.0)
	check(monitor.sent.back() == "STOP", "停止までの秒数を変更できる")
	monitor.queue_free()
	await process_frame
	print("Game reset checks: %s" % ("PASS" if failures == 0 else "%d FAILED" % failures))
	quit(0 if failures == 0 else 1)
