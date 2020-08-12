extends RigidBody

#ALL UNITS ARE S.I. KILOGRAMS-METERS-SECONDS STAMDARD

const DISABLE_GRAVITY = true

#TODO->look into converting these to const (for now requires godot source modification...)
export var MAX_THRUST = 1000#10
export var MIN_THRUST = 0

export var aero_model_file = 'grob103.json'

export var WING_THICKNESS = 0.05 #meters
export var WING_DENSITY = 300 #kilograms / meter^3

const AIR_DENSITY = 1.225 #kg / meter^3
const Cd = 0.04 		#drag coefficient (wikipedia for streamlined body)
const Cl = 70 * Cd		#coefficient of lift (lift to drag ratio is 70)
const Cb = 2 			#blunt surface drag coefficient


#forces applied to the craft
var thrust = 0
var pitch = 0
var roll = 0
var yaw = 0
export var TORQUE_APPLY = 50#0.5
export var THRUST_DELTA = 10#0.1

var origin = null
var density = null
var surfaces: Array = [] #array of surface nodes
#var weights = [] #array of weight nodes



var CoM: Vector3
#var Ix: float
#var Iy: float
#var Iz: float


class Surface:
    var lifting: bool
    var transform: Transform
    var span: float
    var chord: float
    var mass: float
    var mirrored: bool

# Called when the node enters the scene tree for the first time.
func _ready():
    construct_aerodynamics()
    
    #disable gravity scale for debug
    if DISABLE_GRAVITY:
        self.gravity_scale = 0
    


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
    self.add_central_force(get_control_force())
    self.add_torque(get_control_torque())
    for surface in surfaces:
        self.apply_aerodynamics_forces(surface)



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
            KEY_W: change_thrust(THRUST_DELTA)
            KEY_S: change_thrust(-THRUST_DELTA)
            KEY_A: yaw = TORQUE_APPLY if event.pressed else 0
            KEY_D: yaw = -TORQUE_APPLY if event.pressed else 0
            KEY_UP: pitch = TORQUE_APPLY if event.pressed else 0
            KEY_DOWN: pitch = -TORQUE_APPLY if event.pressed else 0
            KEY_LEFT: roll = -TORQUE_APPLY if event.pressed else 0
            KEY_RIGHT: roll = TORQUE_APPLY if event.pressed else 0

#	if event is InputEventKey:
        print("state: thrust=%f," % thrust)
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
    print('torque: ' + String(torque))
    return torque


func apply_aerodynamics_forces(surface: Surface):

    #transform the frame to global coordinate space
    var basis_x = transform.basis.xform(surface.transform.basis.x)
    var basis_y = transform.basis.xform(surface.transform.basis.y)
    var basis_z = transform.basis.xform(surface.transform.basis.z)
    var origin = transform.basis.xform(surface.transform.origin)
    var wing_area = surface.chord * surface.span
    var force = get_surface_force(basis_x, basis_y, basis_z, origin, wing_area, surface.lifting)
    add_force(force, origin)
    
    if surface.mirrored:
        #convert the force and location back to local coordinates
        force = transform.basis.xform_inv(force)
        origin = surface.transform.origin
        
        #mirror over the YZ plane
        force.x *= -1
        origin.x *= -1
        
        #convert back to global coordinates
        force = transform.basis.xform(force)
        origin = transform.basis.xform(origin)
        
        #apply mirrored force
        add_force(force, origin)
        
func get_surface_force(basis_x:Vector3, basis_y:Vector3, basis_z:Vector3, origin:Vector3, wing_area:float, lifting:bool):
    var air_velocity = -(linear_velocity + angular_velocity.cross(origin))
    var air_velocity_x = air_velocity.dot(basis_x)
    var air_velocity_y = air_velocity.dot(basis_y)
    var air_velocity_z = air_velocity.dot(basis_z)

    #absolute magnitudes of aerodynamics forces
    var lift = 0 if not lifting else 1/2 * AIR_DENSITY * air_velocity_z * air_velocity_z * wing_area * Cl
    var drag_z = 0.5 * AIR_DENSITY * air_velocity_z * air_velocity_z * wing_area * Cd
    var drag_x = 0.5 * AIR_DENSITY * air_velocity_x * air_velocity_x * wing_area * Cd
    var drag_y = 0.5 * AIR_DENSITY * air_velocity_y * air_velocity_y * wing_area * Cb
    
#	if air_velocity.length() > 10:
#		pass
    
    #add forces to the center of the wing
    var force = Vector3(0, 0, 0)
    force += lift * basis_y
    force += sign(air_velocity_z) * drag_z * basis_z
    force += sign(air_velocity_x) * drag_x * basis_x
    force += sign(air_velocity_y) * drag_y * basis_y
    return force
    
func load_aerodynamics():
    #read in the json file with the aerodynamics model
    var file = File.new()
    file.open('res://Aerodynamics/' + aero_model_file, File.READ)
    return JSON.parse(file.get_as_text()).result

    
func construct_aerodynamics():
    var model = load_aerodynamics()

    origin = array_to_vector3(model['origin'])
    
    #compute the center of mass based on all masses (and eventually wings
    mass = 1 # kilograms. start with an initial mass since not allowed to be zero
    CoM = Vector3(0, 0, 0)
    var Ix = 0
    var Iy = 0
    var Iz = 0
    
    for weight in model['weights']:
        var m = weight['mass']
        var r = array_to_vector3(weight['root'])
        mass += m
        CoM += m * r
        Ix += (r.y * r.y + r.z * r.z) * m
        Iy += (r.x * r.x + r.z * r.z) * m
        Iz += (r.x * r.x + r.y * r.y) * m
    
    for surface_json in model['surfaces']:
        var surface := build_surface(surface_json)
        surfaces.append(surface)
        var m = surface.mass
        var r = surface.transform.origin
        mass += m
        CoM += m * r
        Ix += (r.y * r.y + r.z * r.z) * m
        Iy += (r.x * r.x + r.z * r.z) * m
        Iz += (r.x * r.x + r.y * r.y) * m
        
        if surface.mirrored:
            mass += m
            r.x *= -1
            CoM += m * r
            Ix += (r.y * r.y + r.z * r.z) * m
            Iy += (r.x * r.x + r.z * r.z) * m
            Iz += (r.x * r.x + r.y * r.y) * m
            
        
    mass -= 1 #remove the initial mass that was added
    CoM /= mass #get the actual CoM position by dividing out all the accumulated mass terms	

    #Adjust moment of inertia based on CoM, using parallel axis theorem
    Ix += (CoM.y * CoM.y + CoM.z * CoM.z) * mass
    Iy += (CoM.x * CoM.x + CoM.z * CoM.z) * mass
    Iz += (CoM.x * CoM.x + CoM.y * CoM.y) * mass
    

    #ajsut all surface origins so that they are relative to the CoM
    for surface in surfaces:
        surface.transform.origin -= CoM
        
    #set the shape's dimensions based on the required moment of inertia
    var collision_shape = self.get_node("CollisionShape")
    var x = sqrt(mass / 12 * (-Ix + Iy + Iz))
    var y = sqrt(mass / 12 * (+Ix - Iy + Iz))
    var z = sqrt(mass / 12 * (+Ix + Iy - Iz))
#	var shape: BoxShape = collision_shape.shape
#	shape.set_extents(Vector3(x, y, z))
    
    #optional adjust shape of visual object
#	var mesh = self.get_node("MeshInstance")
#	mesh.mesh.shape.set_extents(2 * Vector3(x, y, z))
    
    
func build_surface(surface_json) -> Surface:
    #use the json specificiation to build a node containing the surface, to add to the model
    var span_vector = array_to_vector3(surface_json['span'])
    var root_vector = array_to_vector3(surface_json['root'])
    var chord_vector = array_to_vector3(surface_json['chord'])
    
    #create the surface object for holding the wing
    var surface = Surface.new()
    
    #construct the transform representing the origin frame of the wing
    #TODO-> CURRENTLY THIS ASSUMES THAT THE SPAN VECTOR IS IN THE SAME DIRECTION AS THE X VECTOR.... OTHERWISE WILL PRODUCE WING THAT IS UPSIDE DOWN!
    var perp_chord_vector = chord_vector - span_vector.dot(chord_vector) / span_vector.length_squared() * span_vector
    surface.transform.basis = Basis(span_vector.normalized(), span_vector.cross(perp_chord_vector).normalized(), perp_chord_vector.normalized()) #TODO->check if these all need to be normalized first...
    surface.transform.origin = span_vector / 2
    
    #set the rest of the parameters for the wing
    surface.span = span_vector.length()
    surface.chord = (chord_vector - span_vector.dot(chord_vector) / span_vector.length() * span_vector).length()
    surface.mass = surface.span * surface.chord * WING_THICKNESS * WING_DENSITY
    surface.lifting = surface_json['lift']
    surface.mirrored = surface_json['mirror']
    
    return surface
    

    
#Helper function to convert an array of length 3 to a Vector3
func array_to_vector3(arr):
    return Vector3(arr[0], arr[1], arr[2])
    
