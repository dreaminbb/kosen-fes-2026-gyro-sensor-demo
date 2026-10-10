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
# 時定数約21 ms。50 Hz・係数16/秒だった台の追従遅れを短縮する。
const ROTATION_SMOOTHING_RATE := 20.0
const START_POSITION := Vector3(0.0, 0.0, 24.0)
const GOAL_POSITION := Vector3(0.0, 0.0, -26.0)
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

var platform: AnimatableBody3D
var ball: RollingBall
var follow_camera: Camera3D
var target_rotation := Quaternion.IDENTITY
var fall_count := 0
var checkpoint_index := 0
var elapsed_seconds := 0.0
var race_started := false
var completed := false
var keyboard_enabled := true
var overview_enabled := false
var checkpoint_markers: Array[MeshInstance3D] = []
var respawn_position := START_POSITION
var camera_snap := true
var sensor_angular_velocity := Vector3.ZERO


func _ready() -> void:
	_create_platform()
	_create_ball()


func _physics_process(delta: float) -> void:
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
		_drop_ball()
		return
	_update_course_progress(local_ball_position)


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
	if completed:
		return
	var limit := deg_to_rad(MAX_TILT_DEGREES)
	# Pitch/Rollのみ反映。Yawの積分誤差でコースが回り続けるのを防ぐ。
	target_rotation = Basis.from_euler(Vector3(
		clampf(pitch, -limit, limit), 0.0, clampf(roll, -limit, limit)
	)).get_rotation_quaternion()
	if absf(pitch) > 0.02 or absf(roll) > 0.02:
		race_started = true


func set_sensor_angular_velocity(angular_velocity: Vector3) -> void:
	sensor_angular_velocity = angular_velocity
	if is_instance_valid(ball):
		ball.set_sensor_angular_velocity(angular_velocity)


func reset_ball() -> void:
	# 手動リスタートは時間・チェックポイント・落下回数をすべて戻す。
	completed = false
	race_started = false
	elapsed_seconds = 0.0
	fall_count = 0
	checkpoint_index = 0
	respawn_position = START_POSITION
	target_rotation = Quaternion.IDENTITY
	ball.freeze = false
	for marker in checkpoint_markers:
		marker.material_override = _make_material(MOCHA_YELLOW, true)
	_drop_ball()


func get_game_status() -> String:
	if completed:
		return "GOAL!  %.1f秒  |  落下 %d回  —「もう一度」で再挑戦" % [elapsed_seconds, fall_count]
	var stage: String = ["蛇行する壁", "細い橋", "左右にずれる精密橋", "最後の区間 → GOAL"][checkpoint_index]
	return "区間 %d/4: %s  |  CHECKPOINT %d/3  |  %.1f秒  |  落下 %d回" % [
		checkpoint_index + 1, stage, checkpoint_index, elapsed_seconds, fall_count
	]


func _update_course_progress(local_position: Vector3) -> void:
	# 床の上にいる場合だけ通過を認める。落下中に次の区間へ進めない。
	if local_position.y < 0.1 or local_position.y > 1.1:
		return
	if checkpoint_index < CHECKPOINTS.size():
		var checkpoint: Vector3 = CHECKPOINTS[checkpoint_index]
		if local_position.z <= checkpoint.z and absf(local_position.x - checkpoint.x) < 1.5:
			respawn_position = checkpoint
			checkpoint_markers[checkpoint_index].material_override = _make_material(MOCHA_GREEN, true)
			checkpoint_index += 1
	if checkpoint_index == CHECKPOINTS.size():
		var goal_distance := Vector2(local_position.x - GOAL_POSITION.x, local_position.z - GOAL_POSITION.z)
		if goal_distance.length() < 1.0:
			completed = true
			target_rotation = platform.quaternion
			ball.freeze = true


func _drop_ball() -> void:
	# 傾いた台のチェックポイントの上へ、重力方向に1.8 m持ち上げて落とす。
	# 球は台の子にせず、重力・衝突・摩擦だけで運動させる。
	var drop_position := platform.to_global(respawn_position + Vector3(0.0, 0.1, 0.0))
	ball.request_respawn(drop_position + Vector3.UP * BALL_DROP_HEIGHT)
	camera_snap = true


func _create_platform() -> void:
	platform = AnimatableBody3D.new()
	platform.name = "TiltingCourse"
	platform.sync_to_physics = true
	var physics_material := PhysicsMaterial.new()
	physics_material.friction = 0.65
	physics_material.bounce = 0.0
	platform.physics_material_override = physics_material
	add_child(platform)
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
