extends RigidBody

export var aero_model_path = 'res://aerodynamics/test_glider_0.json'

export var WING_DENSITY = 200 #kilograms / meter^3
export var LIFT_TO_DRAG_RATIO = 70

const AIR_DENSITY = 1.225			#kg / meter^3
const Cd = 0.04 					#drag coefficient (wikipedia for streamlined body)
var Cl = LIFT_TO_DRAG_RATIO * Cd	#coefficient of lift (based on lift to drag ratio)


#forces applied to the craft
#TODO->replace torques with control surfaces? alternatively, make torques proportional to speed? control surfaces maybe allow for stall?
var thrust = 0
var pitch = 0
var roll = 0
var yaw = 0
export var TORQUE_APPLY = 50#0.5
export var THRUST_DELTA = 10#0.1


var surfaces = [] #array of surface nodes
var weights = []

class Weight:
	var spatial: Spatial
	var mass: float

class Surface:
	var spatial: Spatial
	var rotZ: float
	var dims: Vector3
	var lift: bool
	var mirrorX: bool
	var mirrorY: bool


# Called when the node enters the scene tree for the first time.
func _ready():
	load_aerodynamics()
	add_surface_geometry()
#	apply_torque_impulse(Vector3(0, 0, 5000))
	

func _integrate_forces(state):
	var net_force = Vector3.ZERO
	var net_torque = Vector3.ZERO
	
	var V = -state.linear_velocity #velocity of air relative to the surface
	var i = 0
	for surface in surfaces:
		surface.spatial.global_transform
		var global_basis_y: Vector3 = surface.spatial.global_transform.basis.y  #normal of the y face
		var area_y = surface.dims.x * surface.dims.z
		var Vy: Vector3 = V.dot(global_basis_y) * global_basis_y #velocity of the air in the y direction (relative to the surface)
		var Fy: Vector3 = 2 * area_y * AIR_DENSITY * Vy.length_squared() * Vy.normalized()
		
		net_force += Fy
		
		var local_torque = surface.spatial.transform.basis.xform(surface.spatial.global_transform.basis.xform_inv(Fy))
		net_torque += surface.spatial.transform.origin.cross(local_torque)
		
	
	state.linear_velocity += (net_force / mass) * state.step
	
#	state.apply_torque_impulse(net_torque * state.step)

func load_aerodynamics():
	var file = File.new()
	file.open(aero_model_path, File.READ)
	var model: Dictionary = JSON.parse(file.get_as_text()).result
	
	#build out surfaces for the model
	for surface_json in model['surfaces']:
		var surface := build_surface(surface_json)

		#add surface unmirrored surface itself to the list
		surfaces.append(surface)

		#add mirrored versions of the surface to the list
		if surface.mirrorX:
			surfaces.append(mirror_surface(surface, true, false))
		if surface.mirrorY:
			surfaces.append(mirror_surface(surface, false, true))
		if surface.mirrorX and surface.mirrorY:
			surfaces.append(mirror_surface(surface, true, true))
		
		#set mirrorX and mirrorY of the original surface to false
		surface.mirrorX = false
		surface.mirrorY = false
		
	for weight_json in model['weights']:
		var weight := Weight.new()
		weight.mass = weight_json['mass']
		var root = array_to_vector3(weight_json['root'])
		weight.spatial = Spatial.new()
		weight.spatial.transform = Transform(Basis.IDENTITY, root)
		weights.append(weight)
		
		

	#compute the center of mass based on all masses (and eventually wings
	var total_mass = 0 # kilograms. start with an initial mass since not allowed to be zero
	var CoM = Vector3(0, 0, 0)
	var Ix = 0
	var Iy = 0
	var Iz = 0

	
	for weight in weights:
		var m = weight.mass
		var r = weight.spatial.transform.origin
		total_mass += m
		CoM += m * r
		Ix += (r.y * r.y + r.z * r.z) * m
		Iy += (r.x * r.x + r.z * r.z) * m
		Iz += (r.x * r.x + r.y * r.y) * m
				
	for surface in surfaces:
		var m = surface.dims.x * surface.dims.y * surface.dims.z * WING_DENSITY
		var r = surface.spatial.transform.origin
		total_mass += m
		CoM += m * r
		Ix += (r.y * r.y + r.z * r.z) * m
		Iy += (r.x * r.x + r.z * r.z) * m
		Iz += (r.x * r.x + r.y * r.y) * m

	mass = total_mass
	CoM /= mass #get the actual CoM position by dividing out all the accumulated mass terms	

	#ajust all surface/weight origins so that CoM is zero, and then attach the spatial node to the rigidbody
	for surface in surfaces:
		surface.spatial.transform.origin -= CoM
		add_child(surface.spatial)
	for weight in weights:
		weight.spatial.transform.origin -= CoM
		add_child(weight.spatial)
			
	#set the mass_simulator shape's dimensions based on the required moment of inertia
	var mass_simulator = $MassSimulator
	var x = sqrt(6 / mass * (-Ix + Iy + Iz))
	var y = sqrt(6 / mass * (+Ix - Iy + Iz))
	var z = sqrt(6 / mass * (+Ix + Iy - Iz))
	mass_simulator.shape.extents = Vector3(x/2, y/2, z/2)
	


func build_surface(surface_json) -> Surface:
	#use the json specificiation to build a node containing the surface, to add to the model
	
	#create the surface object for holding the wing
	var surface = Surface.new()
	
	var root = array_to_vector3(surface_json['root'])
	surface.dims = array_to_vector3(surface_json['dims'])
	surface.rotZ = surface_json['rotZ']
	
	surface.lift = surface_json['lift']
	surface.mirrorX = surface_json['mirrorX']
	surface.mirrorY = surface_json['mirrorY']
	
	#determine the transform of the surface (i.e. the center of the surface, relative to the craft)
	var basis = Basis(Vector3(0, 0, deg2rad(surface.rotZ)))
	var origin = root + basis.xform(Vector3(surface.dims.x / 2, 0, 0))
	surface.spatial = Spatial.new()
	surface.spatial.transform = Transform(basis, origin)
	
	return surface
	
func mirror_surface(surface: Surface, mirrorX: bool, mirrorY: bool) -> Surface:
	var m_surface = Surface.new()
	m_surface.dims = surface.dims
	m_surface.lift = surface.lift
	m_surface.mirrorX = mirrorX
	m_surface.mirrorY = mirrorY
	
	var origin = surface.spatial.transform.origin
	
	#adjust parameters for rotation angle and origin, based on which axis to mirror
	if mirrorX and mirrorY:
		m_surface.rotZ = 180 + surface.rotZ
		origin.x *= -1
		origin.y *= -1
	elif mirrorX:
		m_surface.rotZ = 180 - surface.rotZ
		origin.x *= -1
	elif mirrorY:
		m_surface.rotZ = -surface.rotZ
		origin.y *= -1
	
	#determine the transform of the surface (origin was already computed)
	var basis = Basis(Vector3(0, 0, deg2rad(m_surface.rotZ)))
	m_surface.spatial = Spatial.new()
	m_surface.spatial.transform = Transform(basis, origin)
	
	return m_surface
	
	
func add_surface_geometry():
	for surface in surfaces:
		var geometry := MeshInstance.new()
		geometry.mesh = CubeMesh.new()
		geometry.mesh.size = surface.dims
		geometry.transform = surface.spatial.transform
		$MeshRoot.add_child(geometry)
		
	for weight in weights:
		var geometry := MeshInstance.new()
		geometry.mesh = SphereMesh.new()
		geometry.mesh.radius = 0.5
		geometry.mesh.height = 1.0
		geometry.transform = weight.spatial.transform
		$MeshRoot.add_child(geometry)
		
	
#Helper function to convert an array of length 3 to a Vector3
func array_to_vector3(arr):
	return Vector3(arr[0], arr[1], arr[2])
