extends RigidBody3D

var respawn_pending := false
var respawn_position := Vector3.ZERO
var gyro_visual: Node3D
var sensor_angular_velocity := Vector3.ZERO


func _process(delta: float) -> void:
	if not is_instance_valid(gyro_visual) or sensor_angular_velocity.is_zero_approx():
		return
	var angular_speed := sensor_angular_velocity.length()
	gyro_visual.quaternion = (
		gyro_visual.quaternion
		* Quaternion(sensor_angular_velocity / angular_speed, angular_speed * delta)
	).normalized()


func set_sensor_angular_velocity(angular_velocity: Vector3) -> void:
	sensor_angular_velocity = angular_velocity


func request_respawn(new_position: Vector3) -> void:
	respawn_position = new_position
	respawn_pending = true
	sleeping = false


func hold_at(new_position: Vector3) -> void:
	# freeze中は_integrate_forces()が呼ばれないため、固定時に直接位置を戻す。
	freeze = true
	respawn_pending = false
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	global_transform = Transform3D(Basis.IDENTITY, new_position)
	reset_physics_interpolation()


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if not respawn_pending:
		return
	# 位置・速度を書き換えるのは再出現時だけ。普段の転がりは物理エンジンに任せる。
	state.transform = Transform3D(Basis.IDENTITY, respawn_position)
	state.linear_velocity = Vector3.ZERO
	state.angular_velocity = Vector3.ZERO
	state.sleeping = false
	respawn_pending = false
	# テレポート前後を描画補間しないよう、物理状態の反映後にリセットする。
	reset_physics_interpolation.call_deferred()
