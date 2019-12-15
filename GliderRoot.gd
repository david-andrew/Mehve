extends RigidBody

# Declare member variables here. Examples:
# var a = 2
# var b = "text"

const DISABLE_GRAVITY = true



var thrust = 0
var thrust_delta = 0.1
#const default_force = Vector3(0, 9.8, 0)

#var constant_forces = [Vector3(0, -1, 0)] #array of all forces acting on the glider
#var dynamic_forces = []

# Called when the node enters the scene tree for the first time.
func _ready():
#	self.add_central_force(direction * get_dynamic_force())
#	self.set_linear_velocity(Vector3(0, 1, 0.3))
	
	#disable gravity scale for debug
	if DISABLE_GRAVITY:
		self.gravity_scale = 0
	

#
## Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	self.add_central_force(get_dynamic_force())
#	if Input.is_action_just_pressed("ui_down"):
#		thrust -= 0.25
#	elif Input.is_action_just_pressed("ui_up"):
#		thrust += 0.25
#	elif Input.is_action_just_pressed("ui_cancel"):
#		get_tree().quit()
#	elif Input.is_action_just_pressed("ui_accept"):
#		self.gravity_scale = 1
#	elif Input.is_action_just_released("ui_accept"):
#		self.gravity_scale = 0
		
		
		

func _input(event):
#	print(event.as_text())
	if event.is_action_pressed("ui_page_down"):
		self.gravity_scale = 1
	elif event.is_action_released("ui_page_down"):
		self.gravity_scale = 0
	elif event.is_action_pressed("ui_page_up"):
		self.gravity_scale = -1
	elif event.is_action_released("ui_page_up"):
		self.gravity_scale = 0
	elif event.is_action_pressed("ui_cancel"):
		get_tree().quit()
	elif event.is_action_pressed("ui_up"):
		thrust += thrust_delta
	elif event.is_action_pressed("ui_down"):
		thrust -= thrust_delta
	
	if event is InputEventKey:
		print("state: thrust=%f," % thrust)
		


func get_dynamic_force():
	var force = Vector3(0,0,0)
#	for f in dynamic_forces:
#		force += f
	force += compute_thrust()
	print(force)
	return force
	
func compute_thrust():
	var local_thrust = Vector3(0, 0, thrust)
	return self.global_transform.basis.xform(local_thrust)
#	return self.global_transform.xform(local_thrust)
	
func compute_drag():
	#todo->return a drag vector to apply to the rigidbody
	pass