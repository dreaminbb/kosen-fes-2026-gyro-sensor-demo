extends Node3D

const RollingBall = preload("res://scripts/rolling_ball.gd")
# Godotの1単位を1 mとする。コースはZ=28から-28へ進む。
const COURSE_WIDTH := 8.0
const COURSE_LENGTH := 56.0
const FLOOR_THICKNESS := 0.2
const BALL_RADIUS := 0.22
const BALL_DROP_HEIGHT := 1.8
const FALL_HEIGHT := -3.0
const MAX_TILT_DEGREES := 18.0
# 時定数25 ms。100 Hzのセンサー入力に合わせて応答遅れを抑える。
const ROTATION_SMOOTHING_RATE := 40.0
const START_POSITION := Vector3(0.0, 0.0, 24.0)
const GOAL_POSITION := Vector3(0.0, 0.0, -26.0)
const STAGE_COUNT := 3
const STAGE_2_START := Vector3(0.0, 0.0, 24.0)
const STAGE_2_GOAL := Vector3(0.0, 0.0, -26.0)
# 橋はZ=8から-8までの16 m。10区画を3秒ごとに進める。
const BRIDGE_SEGMENT_COUNT := 10
const BRIDGE_SEGMENT_SECONDS := 3.0
const BRIDGE_ENTRY_Z := 8.0
const BRIDGE_LENGTH := 16.0
const BRIDGE_WIDTH := 2.4
const BRIDGE_SEGMENT_LENGTH := BRIDGE_LENGTH / BRIDGE_SEGMENT_COUNT
const STAGE_2_CHECKPOINTS := [Vector3(0.0, 0.0, 10.0), Vector3(0.0, 0.0, -10.0)]
const STAGE_2_SEESAW_PIVOT_HEIGHT := 0.62
const STAGE_2_SEESAW_MAX_ANGLE := 0.08
const STAGE_2_SEESAW_ENTRY_ANGLE := 0.11
const STAGE_3_START := Vector3(0.0, 0.0, 24.0)
const STAGE_3_GOAL := Vector3(0.0, 0.0, -26.0)
const STAGE_3_POLE_WIDTH := 0.22
const STAGE_3_POLE_LENGTH := 26.0
const STAGE_3_POLE_CENTER_Z := 3.0
const CHECKPOINTS := [
	Vector3(0.0, 0.0, 4.0),
	Vector3(0.0, 0.0, -10.0),
	Vector3(-1.3, 0.0, -22.5),
]
const CAMERA_OFFSET := Vector3(7.0, 12.0, 9.0)
const MOCHA_SURFACE_0 := Color("#313244")
const MOCHA_SURFACE_1 := Color("#45475a")
const MOCHA_SURFACE_2 := Color("#585b70")
const MOCHA_BLUE := Color("#89b4fa")
const MOCHA_TEAL := Color("#94e2d5")
const MOCHA_GREEN := Color("#a6e3a1")
const MOCHA_YELLOW := Color("#f9e2af")
const MOCHA_PEACH := Color("#fab387")
const MOCHA_LAVENDER := Color("#b4befe")
const MOCHA_MAUVE := Color("#cba6f7")

# デバッグ用。ble_monitor.gdから設定される。
var debug_start_stage := 1

var platform: AnimatableBody3D
var ball: RollingBall
var follow_camera: Camera3D
var target_rotation := Quaternion.IDENTITY
var fall_count := 0
var checkpoint_index := 0
var elapsed_seconds := 0.0
var race_started := false
var waiting_for_start := true
var completed := false
var keyboard_enabled := true
var overview_enabled := false
var checkpoint_markers: Array[MeshInstance3D] = []
var respawn_position := START_POSITION
var camera_snap := true
var sensor_angular_velocity := Vector3.ZERO
var stage_index := 0
var stage_root: Node3D
var stage_course_platform: AnimatableBody3D
var bridge_meshes: Array[MeshInstance3D] = []
var bridge_collisions: Array[CollisionShape3D] = []
var bridge_materials: Array[StandardMaterial3D] = []
var bridge_segment_index := 0
var bridge_elapsed := 0.0
var bridge_running := false
var bridge_label: Label3D
var seesaw: AnimatableBody3D
var seesaw_angle := 0.0
var stage_banner: Label3D
var fall_history: Array[Dictionary] = []


func _ready() -> void:
	_create_platform()
	_create_ball()
	reset_ball()


func _physics_process(delta: float) -> void:
	if waiting_for_start:
		return
	if stage_index == 1 and not completed and not ball.respawn_pending:
		_update_disappearing_bridge(delta)
	if keyboard_enabled and not completed:
		# BLE未接続時も方向キーで遊べる。↑は奥（-Z）、←は左（-X）へ転がす。
		var pitch_input := float(Input.is_key_pressed(KEY_DOWN)) - float(Input.is_key_pressed(KEY_UP))
		var roll_input := float(Input.is_key_pressed(KEY_LEFT)) - float(Input.is_key_pressed(KEY_RIGHT))
		set_target_tilt(deg_to_rad(12.0) * pitch_input, deg_to_rad(12.0) * roll_input)
	var next_rotation := platform.quaternion.slerp(
		target_rotation, 1.0 - exp(-ROTATION_SMOOTHING_RATE * delta)
	)
	# 長い台を中央で回すと端の球を跳ね上げるため、球の足元を回転中心とする。
	# 台上の一点を固定して傾けるだけで、球の位置・速度は書き換えない。
	var pivot := platform.to_local(ball.global_position)
	if ball.respawn_pending:
		pivot = respawn_position
	pivot.x = clampf(pivot.x, -COURSE_WIDTH * 0.5, COURSE_WIDTH * 0.5)
	pivot.y = FLOOR_THICKNESS * 0.5
	pivot.z = clampf(pivot.z, -COURSE_LENGTH * 0.5, COURSE_LENGTH * 0.5)
	var fixed_point := platform.transform * pivot
	var next_basis := Basis(next_rotation)
	platform.transform = Transform3D(next_basis, fixed_point - next_basis * pivot)
	if stage_index == 1:
		_update_seesaw(delta)
	if completed:
		return
	if race_started:
		elapsed_seconds += delta
	if ball.respawn_pending:
		return
	# 判定は傾いたコースの座標系で行い、台が傾いても判定位置がずれないようにする。
	var local_ball_position := platform.to_local(ball.global_position)
	if (
		local_ball_position.y < FALL_HEIGHT
		or absf(local_ball_position.x) > COURSE_WIDTH * 0.5 + 3.0
		or absf(local_ball_position.z) > COURSE_LENGTH * 0.5 + 3.0
	):
		fall_count += 1
		fall_history.append({
			"stage": stage_index + 1,
			"section": _get_fall_section(local_ball_position.z),
			"progress": clampf(COURSE_LENGTH * 0.5 - local_ball_position.z, 0.0, COURSE_LENGTH),
			"lateral": local_ball_position.x,
		})
		_drop_ball()
		return
	_update_course_progress(local_ball_position)


func _get_fall_section(course_z: float) -> int:
	if stage_index == 0:
		if course_z > CHECKPOINTS[0].z:
			return 0
		if course_z > CHECKPOINTS[1].z:
			return 1
		if course_z > CHECKPOINTS[2].z:
			return 2
		return 3
	if stage_index == 1:
		if course_z > STAGE_2_CHECKPOINTS[0].z:
			return 0
		if course_z > STAGE_2_CHECKPOINTS[1].z:
			return 1
		return 2
	return 0 if course_z > STAGE_3_POLE_CENTER_Z else 1


func _process(_delta: float) -> void:
	if not is_instance_valid(follow_camera):
		return
	# 球と台の補間済み位置を使う。カメラ自身には追加の追従遅れを付けない。
	var board_transform := platform.get_global_transform_interpolated()
	if overview_enabled:
		follow_camera.size = 66.0
		follow_camera.global_position = board_transform.origin + Vector3(15.0, 50.0, 34.0)
		follow_camera.look_at(board_transform.origin, Vector3.UP)
		return
	follow_camera.size = 12.0
	var ball_position := ball.get_global_transform_interpolated().origin
	var local_position := board_transform.affine_inverse() * ball_position
	# 落下中は視点を台の高さに保ち、再出現する場所を見失わないようにする。
	local_position.y = clampf(local_position.y, 0.3, BALL_DROP_HEIGHT)
	var focus := board_transform * (local_position + Vector3(0.0, 0.0, -2.0))
	follow_camera.global_position = focus + CAMERA_OFFSET
	follow_camera.look_at(focus, Vector3.UP)
	if camera_snap:
		follow_camera.reset_physics_interpolation()
		camera_snap = false


func set_target_tilt(pitch: float, roll: float) -> void:
	if completed or waiting_for_start:
		return
	var limit := deg_to_rad(MAX_TILT_DEGREES)
	# Pitch/Rollのみ反映。Yawの積分誤差でコースが回り続けるのを防ぐ。
	target_rotation = Basis.from_euler(Vector3(
		clampf(pitch, -limit, limit), 0.0, clampf(roll, -limit, limit)
	)).get_rotation_quaternion()


func set_sensor_angular_velocity(angular_velocity: Vector3) -> void:
	sensor_angular_velocity = angular_velocity
	if is_instance_valid(ball):
		ball.set_sensor_angular_velocity(angular_velocity)


func reset_ball() -> void:
	# RESETではスタート上空に固定し、次のボタンとBLEの準備完了を待つ。
	waiting_for_start = true
	completed = false
	race_started = false
	elapsed_seconds = 0.0
	fall_count = 0
	checkpoint_index = 0
	stage_index = clampi(debug_start_stage - 1, 0, STAGE_COUNT - 1)
	_build_stage()
	respawn_position = _get_stage_start_position()
	target_rotation = Quaternion.IDENTITY
	platform.transform = Transform3D.IDENTITY
	platform.reset_physics_interpolation()
	ball.hold_at(respawn_position + Vector3.UP * (FLOOR_THICKNESS * 0.5 + BALL_DROP_HEIGHT))
	set_sensor_angular_velocity(Vector3.ZERO)
	for marker in checkpoint_markers:
		marker.material_override = _make_material(MOCHA_YELLOW, true)
	_update_stage_banner()
	camera_snap = true


func _get_stage_start_position() -> Vector3:
	if stage_index == 1:
		return STAGE_2_START
	if stage_index == 2:
		return STAGE_3_START
	return START_POSITION


func start_ball() -> void:
	if not waiting_for_start:
		return
	waiting_for_start = false
	race_started = true
	ball.freeze = false
	_drop_ball()


func get_game_status() -> String:
	if waiting_for_start:
		return "START上空で待機中  |  ボタンを押して開始"
	if completed:
		return "STAGE %d CLEAR!  %.1f秒  |  落下 %d回" % [stage_index + 1, elapsed_seconds, fall_count]
	var names := ["蛇行コース", "消える橋とシーソー", "細いポール"]
	return "STAGE %d/3: %s  |  %.1f秒  |  落下 %d回" % [stage_index + 1, names[stage_index], elapsed_seconds, fall_count]


func _update_course_progress(local_position: Vector3) -> void:
	# 床の上にいる場合だけ通過を認める。落下中に次の区間へ進めない。
	if local_position.y < 0.1 or local_position.y > 2.0:
		return
	var checkpoints: Array = CHECKPOINTS if stage_index == 0 else (STAGE_2_CHECKPOINTS if stage_index == 1 else [Vector3(0.0, 0.0, STAGE_3_POLE_CENTER_Z)])
	if checkpoint_index < checkpoints.size():
		var checkpoint: Vector3 = checkpoints[checkpoint_index]
		if local_position.z <= checkpoint.z and absf(local_position.x - checkpoint.x) < 1.5:
			respawn_position = checkpoint
			if checkpoint_index < checkpoint_markers.size():
				checkpoint_markers[checkpoint_index].material_override = _make_material(MOCHA_GREEN, true)
			checkpoint_index += 1
	if checkpoint_index == checkpoints.size():
		var goal_position: Vector3 = GOAL_POSITION if stage_index == 0 else (STAGE_2_GOAL if stage_index == 1 else STAGE_3_GOAL)
		var goal_distance := Vector2(local_position.x - goal_position.x, local_position.z - goal_position.z)
		if goal_distance.length() < 1.0:
			_advance_stage()


func _update_stage_banner() -> void:
	if is_instance_valid(stage_banner):
		stage_banner.queue_free()
	if not is_instance_valid(stage_root):
		return
	stage_banner = Label3D.new()
	stage_banner.text = "STAGE %d / 3" % (stage_index + 1)
	stage_banner.position = Vector3(0.0, 0.3, 26.0)
	stage_banner.rotation_degrees.x = -90.0
	stage_banner.font_size = 72
	stage_banner.pixel_size = 0.008
	stage_banner.modulate = MOCHA_BLUE
	platform.add_child(stage_banner)


func _drop_ball() -> void:
	if stage_index == 1:
		_reset_disappearing_bridge()
	# 傾いた台のチェックポイントの上へ、重力方向に1.8 m持ち上げて落とす。
	# 球は台の子にせず、重力・衝突・摩擦だけで運動させる。
	var drop_position := platform.to_global(respawn_position + Vector3(0.0, 0.1, 0.0))
	ball.request_respawn(drop_position + Vector3.UP * BALL_DROP_HEIGHT)
	camera_snap = true


func _create_platform() -> void:
	_build_stage()


func _build_stage() -> void:
	if is_instance_valid(stage_root):
		stage_root.queue_free()
	checkpoint_markers.clear()
	bridge_meshes.clear()
	bridge_collisions.clear()
	bridge_materials.clear()
	bridge_segment_index = 0
	bridge_elapsed = 0.0
	bridge_running = false
	stage_root = Node3D.new()
	stage_root.name = "Stage%d" % (stage_index + 1)
	add_child(stage_root)
	platform = AnimatableBody3D.new()
	platform.name = "TiltingCourse"
	platform.sync_to_physics = true
	var physics_material := PhysicsMaterial.new()
	physics_material.friction = 0.65
	physics_material.bounce = 0.0
	platform.physics_material_override = physics_material
	stage_root.add_child(platform)
	stage_course_platform = platform
	if stage_index == 0:
		_build_stage_one()
	elif stage_index == 1:
		_build_stage_two()
	else:
		_build_stage_three()
	_update_stage_banner()


func _build_stage_one() -> void:
	# 全長56 m。広いスタート、蛇行壁、細い橋、左右にずれる新しい難関、ゴールで構成。
	_add_solid_box(Vector3(8.0, FLOOR_THICKNESS, 14.0), Vector3(0.0, 0.0, 21.0), MOCHA_SURFACE_0)
	_add_solid_box(Vector3(8.0, FLOOR_THICKNESS, 18.0), Vector3(0.0, 0.0, 5.0), MOCHA_SURFACE_1)
	_add_solid_box(Vector3(2.4, FLOOR_THICKNESS, 10.0), Vector3(0.0, 0.0, -8.0), MOCHA_SURFACE_0)
	_add_solid_box(Vector3(8.0, FLOOR_THICKNESS, 6.0), Vector3(0.0, 0.0, -25.0), MOCHA_SURFACE_1)
	# 蛇行する壁は左右の開口を交互に配置する。
	_add_solid_box(Vector3(5.2, 0.7, 0.55), Vector3(-1.4, 0.45, 11.0), MOCHA_PEACH)
	_add_solid_box(Vector3(5.2, 0.7, 0.55), Vector3(1.4, 0.45, 6.5), MOCHA_PEACH)
	_add_solid_box(Vector3(5.2, 0.7, 0.55), Vector3(-1.4, 0.45, 2.0), MOCHA_PEACH)
	# 新しい難関区間。左右へずれた細い足場を連続して渡る。
	_add_solid_box(Vector3(2.8, FLOOR_THICKNESS, 5.5), Vector3(-1.3, 0.0, -13.5), MOCHA_SURFACE_1)
	_add_solid_box(Vector3(2.8, FLOOR_THICKNESS, 5.5), Vector3(1.3, 0.0, -18.0), MOCHA_SURFACE_0)
	_add_solid_box(Vector3(2.8, FLOOR_THICKNESS, 5.5), Vector3(-1.3, 0.0, -22.5), MOCHA_SURFACE_1)
	# 低いガードを広い区間に置き、橋と難関には柱だけを置いて落下しやすくする。
	for side in [-1.0, 1.0]:
		_add_solid_box(Vector3(0.16, 0.45, 14.0), Vector3(side * 3.96, 0.3, 21.0), MOCHA_SURFACE_2)
		_add_solid_box(Vector3(0.16, 0.45, 18.0), Vector3(side * 3.96, 0.3, 5.0), MOCHA_SURFACE_2)
		_add_solid_box(Vector3(0.16, 0.45, 6.0), Vector3(side * 3.96, 0.3, -25.0), MOCHA_SURFACE_2)
	_add_solid_box(Vector3(8.0, 0.45, 0.16), Vector3(0.0, 0.3, 27.95), MOCHA_SURFACE_2)
	_add_solid_box(Vector3(8.0, 0.45, 0.16), Vector3(0.0, 0.3, -27.95), MOCHA_SURFACE_2)
	for offset in [
		Vector3(-0.7, 0.0, -5.0), Vector3(0.7, 0.0, -9.0),
		Vector3(-2.0, 0.0, -13.8), Vector3(2.0, 0.0, -18.1), Vector3(-2.0, 0.0, -22.2),
	]:
		_add_bumper(offset)
	# 床の細線は装飾だけで、球の走行を妨げない。
	for z_position in range(-27, 28):
		if z_position >= -25 and z_position <= -3:
			continue
		var line_width := 2.2 if z_position < -10 else 7.8
		_add_box(platform, Vector3(line_width, 0.006, 0.018), Vector3(0.0, 0.105, z_position), MOCHA_SURFACE_2)
	for checkpoint in CHECKPOINTS:
		var marker_width := 2.6 if checkpoint.z < -10.0 else (2.2 if checkpoint.z < 0.0 else 7.6)
		var marker := _add_box(platform, Vector3(marker_width, 0.01, 0.15), Vector3(checkpoint.x, 0.11, checkpoint.z), MOCHA_YELLOW)
		marker.material_override = _make_material(MOCHA_YELLOW, true)
		checkpoint_markers.append(marker)
	_add_surface_ring(START_POSITION, MOCHA_BLUE, 1.1)
	_add_surface_ring(GOAL_POSITION, MOCHA_GREEN, 1.1)
	_add_course_label("START", START_POSITION + Vector3(0.0, 0.15, 0.0), MOCHA_LAVENDER)
	_add_course_label("GOAL", GOAL_POSITION + Vector3(0.0, 0.15, 0.0), MOCHA_GREEN)
	_add_course_label("1", Vector3(-3.0, 0.15, 4.5), MOCHA_YELLOW)
	_add_course_label("2", Vector3(-0.7, 0.15, -10.0), MOCHA_YELLOW)
	_add_course_label("3  CHICANE", Vector3(-1.3, 0.15, -22.5), MOCHA_YELLOW)


func _build_stage_two() -> void:
	# 全長56 m。ピストンを外し、入口、消える橋、出口、シーソーを接続する。
	_add_solid_box(Vector3(8.0, FLOOR_THICKNESS, 20.0), Vector3(0.0, 0.0, 18.0), MOCHA_SURFACE_0)
	_add_solid_box(Vector3(5.0, FLOOR_THICKNESS, 4.0), Vector3(0.0, 0.0, -10.0), MOCHA_SURFACE_1)
	_add_solid_box(Vector3(5.0, FLOOR_THICKNESS, 6.0), Vector3(0.0, 0.0, -25.0), MOCHA_SURFACE_1)
	_add_seesaw_entry_ramp()
	_build_seesaw()
	for index in range(BRIDGE_SEGMENT_COUNT):
		var offset := Vector3(0.0, 0.0, BRIDGE_ENTRY_Z - (index + 0.5) * BRIDGE_SEGMENT_LENGTH)
		var size := Vector3(BRIDGE_WIDTH, FLOOR_THICKNESS, BRIDGE_SEGMENT_LENGTH)
		var collision := CollisionShape3D.new()
		var shape := BoxShape3D.new()
		shape.size = size
		collision.shape = shape
		collision.position = offset
		collision.disabled = index > 1
		platform.add_child(collision)
		bridge_collisions.append(collision)
		var mesh := _add_box(platform, size, offset, MOCHA_MAUVE)
		mesh.visible = index <= 1
		var material := _make_material(MOCHA_MAUVE)
		if index == 1:
			material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
			material.albedo_color.a = 0.32
		mesh.material_override = material
		bridge_meshes.append(mesh)
		bridge_materials.append(material)
	bridge_label = Label3D.new()
	bridge_label.position = Vector3(0.0, 0.16, 12.0)
	bridge_label.rotation_degrees.x = -90.0
	bridge_label.font_size = 48
	bridge_label.pixel_size = 0.008
	bridge_label.modulate = MOCHA_MAUVE
	platform.add_child(bridge_label)
	_reset_disappearing_bridge()
	_add_surface_ring(STAGE_2_START, MOCHA_BLUE, 1.1)
	_add_course_label("START", STAGE_2_START + Vector3(0.0, 0.15, 0.0), MOCHA_LAVENDER)
	_add_surface_ring(STAGE_2_GOAL, MOCHA_GREEN, 1.1)
	_add_course_label("GOAL", STAGE_2_GOAL + Vector3(0.0, 0.15, 0.0), MOCHA_GREEN)
	for checkpoint in STAGE_2_CHECKPOINTS:
		checkpoint_markers.append(_add_box(platform, Vector3(4.5, 0.01, 0.15), checkpoint + Vector3.UP * 0.11, MOCHA_YELLOW))


func _build_stage_three() -> void:
	_add_solid_box(Vector3(8.0, FLOOR_THICKNESS, 12.0), Vector3(0.0, 0.0, 22.0), MOCHA_SURFACE_0)
	_add_solid_box(Vector3(STAGE_3_POLE_WIDTH, FLOOR_THICKNESS, STAGE_3_POLE_LENGTH), Vector3(0.0, 0.0, STAGE_3_POLE_CENTER_Z), MOCHA_TEAL)
	_add_solid_box(Vector3(4.0, FLOOR_THICKNESS, 18.0), Vector3(0.0, 0.0, -19.0), MOCHA_SURFACE_1)
	_add_surface_ring(STAGE_3_START, MOCHA_BLUE, 1.1)
	_add_course_label("START", STAGE_3_START + Vector3(0.0, 0.15, 0.0), MOCHA_LAVENDER)
	_add_surface_ring(STAGE_3_GOAL, MOCHA_GREEN, 1.1)
	_add_course_label("GOAL", STAGE_3_GOAL + Vector3(0.0, 0.15, 0.0), MOCHA_GREEN)
	checkpoint_markers.append(_add_box(platform, Vector3(0.3, 0.01, 0.15), Vector3(0.0, 0.11, STAGE_3_POLE_CENTER_Z), MOCHA_YELLOW))


func _build_seesaw() -> void:
	seesaw_angle = 0.0
	seesaw = AnimatableBody3D.new()
	seesaw.name = "Stage2Seesaw"
	seesaw.sync_to_physics = true
	var physics_material := PhysicsMaterial.new()
	physics_material.friction = 0.8
	seesaw.physics_material_override = physics_material
	stage_root.add_child(seesaw)
	var size := Vector3(0.55, FLOOR_THICKNESS, 10.0)
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = size
	collision.shape = shape
	seesaw.add_child(collision)
	_add_box(seesaw, size, Vector3.ZERO, MOCHA_TEAL)
	seesaw.global_transform = platform.global_transform * Transform3D(
		Basis.IDENTITY, Vector3(0.0, STAGE_2_SEESAW_PIVOT_HEIGHT, -17.0)
	)
	# 中央の支点をコース上に描き、シーソーであることがわかるようにする。
	var fulcrum := CylinderMesh.new()
	fulcrum.top_radius = 0.42
	fulcrum.bottom_radius = 0.12
	fulcrum.height = 0.55
	var fulcrum_mesh := MeshInstance3D.new()
	fulcrum_mesh.mesh = fulcrum
	fulcrum_mesh.position = Vector3(0.0, STAGE_2_SEESAW_PIVOT_HEIGHT * 0.5, -17.0)
	fulcrum_mesh.material_override = _make_material(MOCHA_YELLOW)
	platform.add_child(fulcrum_mesh)
	# _add_course_label("SEESAW", Vector3(0.0, STAGE_2_SEESAW_PIVOT_HEIGHT + 0.2, -17.0), MOCHA_YELLOW)


func _add_seesaw_entry_ramp() -> void:
	var size := Vector3(0.55, FLOOR_THICKNESS, 2.0)
	var ramp_center := Vector3(0.0, 0.11, -11.0)
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = size
	collision.shape = shape
	collision.position = ramp_center
	collision.rotation.x = STAGE_2_SEESAW_ENTRY_ANGLE
	platform.add_child(collision)
	var ramp_mesh := _add_box(platform, size, ramp_center, MOCHA_YELLOW)
	ramp_mesh.rotation.x = STAGE_2_SEESAW_ENTRY_ANGLE


func _update_seesaw(delta: float) -> void:
	if not is_instance_valid(seesaw):
		return
	var local_ball := platform.to_local(ball.global_position)
	var relative_z := local_ball.z + 17.0
	var target_angle := 0.0
	if absf(relative_z) < 8.0 and absf(local_ball.x) < 0.55:
		target_angle = clampf(relative_z * 0.04, -STAGE_2_SEESAW_MAX_ANGLE, STAGE_2_SEESAW_MAX_ANGLE)
	seesaw_angle = lerpf(seesaw_angle, target_angle, 1.0 - exp(-8.0 * delta))
	var local_transform := Transform3D(
		Basis(Vector3.RIGHT, seesaw_angle),
		Vector3(0.0, STAGE_2_SEESAW_PIVOT_HEIGHT, -17.0)
	)
	seesaw.global_transform = platform.global_transform * local_transform


func _advance_stage() -> void:
	if stage_index >= STAGE_COUNT - 1:
		completed = true
		target_rotation = platform.quaternion
		ball.freeze = true
		return
	stage_index += 1
	checkpoint_index = 0
	completed = false
	target_rotation = Quaternion.IDENTITY
	_build_stage()
	platform.transform = Transform3D.IDENTITY
	platform.reset_physics_interpolation()
	respawn_position = STAGE_2_START if stage_index == 1 else STAGE_3_START
	ball.hold_at(respawn_position + Vector3.UP * (FLOOR_THICKNESS * 0.5 + BALL_DROP_HEIGHT))
	ball.freeze = false
	_drop_ball()
	camera_snap = true


func _reset_disappearing_bridge() -> void:
	bridge_elapsed = 0.0
	bridge_running = false
	_show_bridge_segment(0)


func _show_bridge_segment(index: int) -> void:
	bridge_segment_index = index
	for part in range(bridge_meshes.size()):
		var is_current := part == index
		var is_next := part == index + 1 and part < BRIDGE_SEGMENT_COUNT
		bridge_meshes[part].visible = is_current or is_next
		bridge_collisions[part].disabled = not (is_current or is_next)
		bridge_materials[part].albedo_color.a = 1.0 if is_current else 0.32
		bridge_materials[part].transparency = (
			BaseMaterial3D.TRANSPARENCY_DISABLED if is_current
			else BaseMaterial3D.TRANSPARENCY_ALPHA
		)
	bridge_label.text = "BRIDGE  %d / %d  ·  3 SEC" % [index + 1, BRIDGE_SEGMENT_COUNT]


func _update_disappearing_bridge(delta: float) -> void:
	var local_position := platform.to_local(ball.global_position)
	var segment_center := bridge_collisions[bridge_segment_index].position
	var on_bridge := (
		absf(local_position.x) < BRIDGE_WIDTH * 0.5
		and absf(local_position.z - segment_center.z) < BRIDGE_SEGMENT_LENGTH * 0.5
		and absf(local_position.y - (FLOOR_THICKNESS * 0.5 + BALL_RADIUS)) < 0.12
	)
	# 最初の区画に乗った時点から計時。10区画目は27秒後に現れる。
	if not bridge_running:
		if not on_bridge:
			return
		bridge_running = true
	bridge_elapsed += delta
	var next_index := bridge_segment_index + 1
	# 消える直前の1秒間、現在の区画を点滅させて切り替えの拍を知らせる。
	if bridge_elapsed >= BRIDGE_SEGMENT_SECONDS - 1.0 and bridge_elapsed < BRIDGE_SEGMENT_SECONDS:
		var blink_phase := int(floor((bridge_elapsed - (BRIDGE_SEGMENT_SECONDS - 1.0)) * 4.0))
		bridge_meshes[bridge_segment_index].visible = blink_phase % 2 == 0
	# 区画が消える直前に次の区画を薄く見せ、先に当たり判定を有効にする。
	if next_index < BRIDGE_SEGMENT_COUNT and bridge_elapsed >= BRIDGE_SEGMENT_SECONDS - 0.5:
		bridge_meshes[next_index].visible = true
		bridge_collisions[next_index].disabled = false
	if bridge_elapsed < BRIDGE_SEGMENT_SECONDS:
		return
	bridge_elapsed -= BRIDGE_SEGMENT_SECONDS
	if bridge_segment_index == BRIDGE_SEGMENT_COUNT - 1:
		_reset_disappearing_bridge()
		return
	_show_bridge_segment(next_index)


func _create_ball() -> void:
	ball = RollingBall.new()
	ball.name = "RollingBall"
	ball.position = START_POSITION + Vector3.UP * BALL_DROP_HEIGHT
	ball.mass = 0.5 # kg
	ball.continuous_cd = true
	ball.linear_damp = 0.08
	ball.angular_damp = 0.12
	var physics_material := PhysicsMaterial.new()
	physics_material.friction = 0.65
	physics_material.bounce = 0.1
	ball.physics_material_override = physics_material
	var collision := CollisionShape3D.new()
	var shape := SphereShape3D.new()
	shape.radius = BALL_RADIUS
	collision.shape = shape
	ball.add_child(collision)
	var visual := Node3D.new()
	visual.name = "BallVisual"
	ball.add_child(visual)
	var mesh_instance := MeshInstance3D.new()
	var mesh := SphereMesh.new()
	mesh.radius = BALL_RADIUS
	mesh.height = BALL_RADIUS * 2.0
	mesh_instance.mesh = mesh
	mesh_instance.material_override = _make_material(MOCHA_PEACH)
	visual.add_child(mesh_instance)
	# 3本の球面リングが物理回転とIMU角速度の両方を見せる。
	for ring_data in [
		[MOCHA_LAVENDER, Vector3.ZERO],
		[MOCHA_BLUE, Vector3(PI / 2.0, 0.0, 0.0)],
		[MOCHA_TEAL, Vector3(0.0, 0.0, PI / 2.0)],
	]:
		var ring := MeshInstance3D.new()
		var ring_mesh := TorusMesh.new()
		ring_mesh.inner_radius = 0.162
		ring_mesh.outer_radius = 0.19
		ring.mesh = ring_mesh
		ring.rotation = ring_data[1]
		ring.material_override = _make_material(ring_data[0], true)
		visual.add_child(ring)
	ball.gyro_visual = visual
	add_child(ball)


func _add_solid_box(size: Vector3, offset: Vector3, color: Color) -> void:
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = size
	collision.shape = shape
	collision.position = offset
	platform.add_child(collision)
	_add_box(platform, size, offset, color)


func _add_bumper(offset: Vector3) -> void:
	var collision := CollisionShape3D.new()
	var shape := CylinderShape3D.new()
	shape.radius = 0.55
	shape.height = 0.8
	collision.shape = shape
	collision.position = offset + Vector3.UP * 0.5
	platform.add_child(collision)
	var mesh_instance := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.top_radius = 0.55
	mesh.bottom_radius = 0.55
	mesh.height = 0.8
	mesh_instance.mesh = mesh
	mesh_instance.position = collision.position
	mesh_instance.material_override = _make_material(MOCHA_MAUVE)
	platform.add_child(mesh_instance)


func _add_surface_ring(offset: Vector3, color: Color, radius: float) -> void:
	var mesh_instance := MeshInstance3D.new()
	var mesh := TorusMesh.new()
	mesh.inner_radius = radius - 0.055
	mesh.outer_radius = radius
	mesh_instance.mesh = mesh
	mesh_instance.position = offset + Vector3.UP * 0.115
	mesh_instance.scale.y = 0.12
	mesh_instance.material_override = _make_material(color, true)
	platform.add_child(mesh_instance)


func _add_course_label(text: String, offset: Vector3, color: Color) -> void:
	var label := Label3D.new()
	label.text = text
	label.position = offset
	label.rotation_degrees.x = -90.0
	label.font_size = 64
	label.pixel_size = 0.008
	label.modulate = color
	label.outline_modulate = Color("0f172a")
	platform.add_child(label)


func _add_box(parent: Node3D, size: Vector3, offset: Vector3, color: Color) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mesh_instance.mesh = mesh
	mesh_instance.position = offset
	mesh_instance.material_override = _make_material(color)
	parent.add_child(mesh_instance)
	return mesh_instance


func _make_material(color: Color, glowing: bool = false) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.metallic = 0.12
	material.roughness = 0.42
	if glowing:
		material.emission_enabled = true
		material.emission = color
		material.emission_energy_multiplier = 0.6
	return material
