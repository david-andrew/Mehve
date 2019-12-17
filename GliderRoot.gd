extends RigidBody




const DISABLE_GRAVITY = true

#TODO->look into converting these to const (for now requires godot source modification...)
export var MAX_THRUST = 1
export var MIN_THRUST = 0

export var aero_model_file = 'grob103.json'


#forces applied to the craft
var thrust = 0
var pitch = 0
var roll = 0
var yaw = 0
export var TORQUE_APPLY = 0.5

export var THRUST_DELTA = 0.1



# Called when the node enters the scene tree for the first time.
func _ready():
	construct_aerodynamics()
	
#	self.add_central_force(direction * get_dynamic_force())
#	self.set_linear_velocity(Vector3(0, 1, 0.3))
	
	#disable gravity scale for debug
	if DISABLE_GRAVITY:
		self.gravity_scale = 0
	

#
## Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	self.add_central_force(get_control_force())
	self.add_torque(get_control_torque())


func _input(event):	
	#handle regular actions
	if event.is_action_pressed("ui_cancel"):
		get_tree().quit()
	elif event.is_action_pressed("ui_page_down"):
		self.gravity_scale = 1
	elif event.is_action_released("ui_page_down"):
		self.gravity_scale = 0
	elif event.is_action_pressed("ui_page_up"):
		self.gravity_scale = -1
	elif event.is_action_released("ui_page_up"):
		self.gravity_scale = 0
	
	
	#because not enough actions for 8-DoF, using raw keypresses
	if event is InputEventKey:
		match event.scancode:
			KEY_W:
				change_thrust(THRUST_DELTA)
			KEY_A:
				yaw = TORQUE_APPLY if event.pressed else 0
			KEY_S:
				change_thrust(-THRUST_DELTA)
			KEY_D:
				yaw = -TORQUE_APPLY if event.pressed else 0
			KEY_UP:
				pitch = TORQUE_APPLY if event.pressed else 0
			KEY_DOWN:
				pitch = -TORQUE_APPLY if event.pressed else 0
			KEY_LEFT:
				roll = -TORQUE_APPLY if event.pressed else 0
			KEY_RIGHT:
				roll = TORQUE_APPLY if event.pressed else 0


	
#	if event is InputEventKey:
#		print("state: thrust=%f," % thrust)
#

func change_thrust(delta):
	thrust = clamp(thrust + delta, MIN_THRUST, MAX_THRUST)

func get_control_force():
	var local_force = Vector3(0, 0, thrust)
	var force = self.global_transform.basis.xform(local_force)
#	print('force: ' + String(force))
	return force
	
func get_control_torque():
	var local_torque = Vector3(pitch, yaw, roll)
	var torque = self.global_transform.basis.xform(local_torque)
#	print('torque: ' + String(torque))
	return torque

#func get_dynamic_force():
#	var force = Vector3(0,0,0)
##	for f in dynamic_forces:
##		force += f
#	force += compute_thrust()
#	print(force)
#	return force
	

	

func compute_drag():
	#todo->return a drag vector to apply to the rigidbody
	pass
	
	
	
func load_aerodynamics():
	#read in the json file with the aerodynamics model
	var file = File.new()
	file.open('res://Aerodynamics/' + aero_model_file, File.READ)
	return JSON.parse(file.get_as_text()).result

	
func construct_aerodynamics():
	var model = load_aerodynamics()
	

	var origin = array_to_vector3(model['origin'])
	var density = model['density']
	for surface in model['surfaces']:
		print(surface)
	for weight in model['weights']:
		print(weight)
	
	
	#use the loaded aerodynamics model to construct the variables and physics objects for 
	pass
	
func array_to_vector3(arr):
	return Vector3(arr[0], arr[1], arr[2])
	