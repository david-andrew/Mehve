extends Spatial


var altitude = 0
var azimuth = 0

const HSPEED = 1.0
const VSPEED = 1.0

# Called when the node enters the scene tree for the first time.
func _ready():
	var euler: Vector3 = global_transform.basis.get_euler()
	altitude = euler.x
	azimuth = euler.y
	


func _process(delta):
	#get input, modify azimuth and altitude
	var h_input = Input.get_action_strength("camera_orbit_left") - Input.get_action_strength("camera_orbit_right")
	var v_input = (Input.get_action_strength("camera_orbit_up") - Input.get_action_strength("camera_orbit_down"))

	altitude -= v_input * VSPEED * delta
	azimuth -= h_input * HSPEED * delta
	altitude = clamp(altitude, deg2rad(-80), deg2rad(80))

	global_transform.basis = Basis(Vector3(altitude, azimuth, 0))
