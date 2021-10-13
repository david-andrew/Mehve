extends ImmediateGeometry
#TODO->load in aerodynamic surfaces


#func _ready():
#	pass # Replace with function body.


func draw_rect(root: Transform, chord: float, aspect: float):
	begin(Mesh.PRIMITIVE_TRIANGLES)

	#aspect = chord / width
	var width = chord / aspect
	var x0 = root.origin
	var x1 = x0 + root.basis.xform(Vector3(chord, 0, 0))
	var x3 = x0 + root.basis.xform(Vector3(0, 0, width))
	var x2 = x1 + x3 - x0
	var normal = x3.cross(x1)
	


	#counter-clockwise order for top
	set_normal(normal)
	add_vertex(x0)
	add_vertex(x1)
	add_vertex(x2)
	
	add_vertex(x0)
	add_vertex(x2)
	add_vertex(x3)

	#clockwise order for bottom
	set_normal(-normal)
	add_vertex(x0)
	add_vertex(x2)
	add_vertex(x1)
	
	add_vertex(x0)
	add_vertex(x3)
	add_vertex(x2)
	
	end()
	

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	# Clean up before drawing.
	clear()
	draw_rect(Transform(self.transform.basis, Vector3(2, 0, 0)), 2, 2)
#	# Begin draw.
#	begin(Mesh.PRIMITIVE_TRIANGLES)
#
#	# Prepare attributes for add_vertex.
##	set_normal(Vector3(0, 0, 1))
##	set_uv(Vector2(0, 0))
#	# Call last for each vertex, adds the above attributes.
#	add_vertex(Vector3(-1, -1, 0))
#
##	set_normal(Vector3(0, 0, 1))
##	set_uv(Vector2(0, 1))
#	add_vertex(Vector3(-1, 1, 0))
#
##	set_normal(Vector3(0, 0, 1))
##	set_uv(Vector2(1, 1))
#	add_vertex(Vector3(1, 1, 0))
#
#	add_vertex(Vector3(2, 1, 0))
#	add_vertex(Vector3(1, -1, 0))
#	add_vertex(Vector3(-1, -1, 0))
#
#	# End drawing.
#	end()
