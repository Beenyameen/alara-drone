extends SceneTree

func _init():
    var mm = MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.instance_count = 1
    
    mm.set_instance_transform(0, Transform3D(Basis(), Vector3(10, 20, 30)))
    mm.set_instance_color(0, Color(1, 2, 3, 4))
    
    var buf = mm.buffer
    for i in range(buf.size()):
        print("buf[" + str(i) + "] = " + str(buf[i]))
    quit()
