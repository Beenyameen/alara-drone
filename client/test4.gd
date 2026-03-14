extends SceneTree

func _init():
    var mm = MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.instance_count = 1
    
    # We set specifically unique numbers to trace them!
    # Basis:
    # Row 0: 10, 11, 12
    # Row 1: 20, 21, 22
    # Row 2: 30, 31, 32
    # Origin: 40, 41, 42
    mm.set_instance_transform(0, Transform3D(Basis(Vector3(10, 20, 30), Vector3(11, 21, 31), Vector3(12, 22, 32)), Vector3(40, 41, 42)))
    
    # Color: R=0.1, G=0.2, B=0.3, A=0.4
    mm.set_instance_color(0, Color(0.1, 0.2, 0.3, 0.4))
    
    var buf = mm.buffer
    var file = FileAccess.open("res://buffer_output.txt", FileAccess.WRITE)
    for i in range(buf.size()):
        file.store_line("buf[" + str(i) + "] = " + str(buf[i]))
    file.close()
    
    quit()
