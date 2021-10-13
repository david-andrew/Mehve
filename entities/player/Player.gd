extends KinematicBody


const GROUND_SPEED = 10
const RUN_SPEED = 20
const GROUND_ACCELERATION = 30
const AIR_ACCELERATION = 10
const H_ANGULAR_SPEED = 5
const JUMP_IMPULSE = 10
const GRAVITY = 9.8 #should be pulled from some sort of global



#player input movement
var forward = 0
var sideways = 0
var vertical = 0
var angle = 0
var run = false
var grounded = false


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	forward = Input.get_action_strength("3rd_person_forward") - Input.get_action_strength("3rd_person_backward")
	sideways = Input.get_action_strength("3rd_person_right") - Input.get_action_strength("3rd_person_left")
	run = Input.is_action_pressed("3rd_person_run")

#	grounded = is_on_floor()
#	if not grounded:
#		vertical -= GRAVITY * delta
#	else:
#		vertical = 0

	var speed = RUN_SPEED if run else GROUND_SPEED
	
	var camera_azimuth = $OrbitCamera.azimuth
	var strafe = Vector2(forward, sideways).clamped(1.0).rotated(-camera_azimuth + 3*3.14159/2)
	if strafe.length_squared() > 0.01:
		angle = strafe.angle_to(Vector2.RIGHT)
#	var direction = clamp_vec(Vector3(forward, 0, sideways))

	var motion = convert_strafe_to_3d(strafe) * speed + Vector3.UP * vertical

	move_and_collide(motion * delta)
	self.transform.basis = Basis.IDENTITY.rotated(Vector3.UP, angle)


func clamp_vec(vector, value=1.0):
	var length = vector.length()
	if length > value:
		return vector * value/length
	else:
		return vector



func convert_strafe_to_3d(strafe: Vector2) -> Vector3:
	return Vector3(strafe.x, 0, strafe.y)

func convert_3d_to_strafe(vec: Vector3) -> Vector2:
	return Vector2(vec.x, vec.z)
