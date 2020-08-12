extends RigidBody

export var aero_model_path = 'res://aerodynamics/test_glider_3.json'

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

const ROLL_STRENGTH = 5000
const PITCH_STRENGTH = 10000
const YAW_STRENGTH = 5000
const THRUST_STRENGHT = 5000


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
#    apply_torque_impulse(Vector3.BACK * 10000)
#	apply_central_impulse()
    linear_velocity = global_transform.basis.xform(Vector3.FORWARD) * 50
#    angular_velocity = global_transform.basis.xform(Vector3.BACK) * 20
    
func _process(delta):
    pitch = Input.get_action_strength("player_pitch_up") - Input.get_action_strength("player_pitch_down")
    thrust = Input.get_action_strength("player_thrust_up") - Input.get_action_strength("player_thrust_down")
    roll = Input.get_action_strength("player_roll_left") - Input.get_action_strength("player_roll_right")
    yaw = Input.get_action_strength("player_yaw_left") - Input.get_action_strength("player_yaw_right")

    if randf() < 0.1:
        print("energy: ", get_energy())

#	if abs(pitch) < 0.1:
#		pitch = 0
#	if abs(roll) < 0.1:
#		roll = 0
#	if abs(yaw) < 0.1:
#		yaw = 0
    pass


func _integrate_forces(state):
    var net_force = Vector3.ZERO
    var net_torque = Vector3.ZERO
    
    var V_CoM = -state.linear_velocity  # linear velocity of air relative to the CoM
    var W = -state.angular_velocity		# angular velocity of air
    
    for surface in surfaces:
        var forces = get_surface_forces(surface, V_CoM, W)
        var F: Vector3 = forces[0]
        var T: Vector3 = forces[1]
        
        net_force += F
        net_torque += T
        
    
    
    #player input control. TODO->convert these to control surfaces
    var roll_torque: Vector3 = global_transform.basis.xform(Vector3.BACK) * roll * ROLL_STRENGTH
    var yaw_torque: Vector3 = global_transform.basis.xform(Vector3.UP) * yaw * YAW_STRENGTH
    var pitch_torque: Vector3 = global_transform.basis.xform(Vector3.RIGHT) * pitch * PITCH_STRENGTH
    
    net_torque += roll_torque + yaw_torque + pitch_torque
    
#	state.linear_velocity += (net_force / mass) * state.step
    state.apply_central_impulse(net_force * state.step)
    state.apply_torque_impulse(net_torque * state.step)
    
    

#	state.apply_torque_impulse((roll_torque + yaw_torque + pitch_torque) * state.step)


func get_energy():
    return mass * gravity_scale * global_transform.origin.y + 0.5 * mass * linear_velocity.length_squared()

# compute the force and torque at the CoM for the given surface
func get_surface_forces(surface: Surface, V_CoM: Vector3, W: Vector3):
    #tasks:
    # y face linear blunt force + torque
    # y face angular blunt force + torque
    # other faces blunt...
    
    # -z linear lift + drag
    # angular lift + drag?
    # etc
    # other faces?
    

    # vector from CoM to center of surface	
    var r: Vector3 = surface.spatial.global_transform.origin - global_transform.origin
    
    # linear velocity at origin of the surface (CoM linear + CoM angular x radius)
    var V = V_CoM + W.cross(r)
    
    #areas to be used for calculations	
    var area_y = surface.dims.x * surface.dims.z
    var area_x = surface.dims.y * surface.dims.z
    var area_z = surface.dims.x * surface.dims.y

    #unit vectors for the orientation of the surface origin frame	
    var basis_x: Vector3 = surface.spatial.global_transform.basis.x
    var basis_y: Vector3 = surface.spatial.global_transform.basis.y
    var basis_z: Vector3 = surface.spatial.global_transform.basis.z
    
    
    # component linear velocities at the origin of the surface
    var Vx: Vector3 = V.dot(basis_x) * basis_x
    var Vy: Vector3 = V.dot(basis_y) * basis_y
    var Vz: Vector3 = V.dot(basis_z) * basis_z
    
    # component angular velocities
    var Wx: Vector3 = W.dot(basis_x) * basis_x
    var Wy: Vector3 = W.dot(basis_y) * basis_y
    var Wz: Vector3 = W.dot(basis_z) * basis_z
    
    
    # y face blunt force + torque
    var Fyy_blunt: Vector3 = 2 * area_y * AIR_DENSITY * Vy.length_squared() * Vy.normalized()
    var Tyy_blunt: Vector3 = r.cross(Fyy_blunt)
    var Tyz_blunt: Vector3 = pow(surface.dims.x, 4) / 16.0 * surface.dims.z * AIR_DENSITY * Wz.length_squared() * Wz.normalized() 
    var Tyx_blunt: Vector3 = pow(surface.dims.z, 4) / 16.0 * surface.dims.x * AIR_DENSITY * Wx.length_squared() * Wx.normalized()
    var Fy_blunt = Fyy_blunt + Tyz_blunt.cross(2*r) + Tyx_blunt.cross(2*r)
       
    
    # y face lift force + torque
#    var Vz_CoM =  V_CoM.dot(basis_z) * basis_z
    var Fy_lift: Vector3 = 0.5 * Cl * AIR_DENSITY * area_y * Vz.length_squared() * basis_y if surface.lift else Vector3.ZERO
    if surface.mirrorX:
        Fy_lift *= -1
    var Ty_lift: Vector3 = r.cross(Fy_lift)
    
    # y face drag force + torque
#    var Fy_drag: Vector3 = 0.5 * Cd * AIR_DENSITY * area_y * Vz.length_squared() * Vz.normalized() if surface.lift else Vector3.ZERO
    var Fy_drag: Vector3 = Fy_lift.length() / LIFT_TO_DRAG_RATIO * Vz.normalized()
    var Ty_drag: Vector3 = r.cross(Fy_drag)
    
    
#    #debug
#    if surface.lift and randf() < 0.1:
#        print(Fy_lift)
#
#
    
    
    var F: Vector3 = Fy_blunt# + Fy_lift + Fy_drag
    var T: Vector3 = Tyy_blunt + Tyz_blunt + Tyx_blunt# + Ty_lift + Ty_drag 
    return [F, T]



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
