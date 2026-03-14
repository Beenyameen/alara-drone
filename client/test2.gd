extends SceneTree

func _init():
    var mm = MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.instance_count = 1
    
    mm.set_instance_transform(0, Transform3D(Basis(Vector3(1, 2, 3), Vector3(4, 5, 6), Vector3(7, 8, 9)), Vector3(10, 20, 30)))
    mm.set_instance_color(0, Color(0.1, 0.2, 0.3, 0.4))
    
    var buf = mm.buffer
    var file = FileAccess.open("user://gd_layout.txt", FileAccess.WRITE)
    for i in range(buf.size()):
        file.store_line("buf[" + str(i) + "] = " + str(buf[i]))
    file.close()
    
    # Also print the path so we know where it is
    var path_file = FileAccess.open("res://gd_path.txt", FileAccess.WRITE)
    path_file.store_line(ProjectSettings.globalize_path("user://gd_layout.txt"))
    path_file.close()
    
    quit()
