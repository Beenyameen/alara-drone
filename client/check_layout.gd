extends SceneTree

func _init():
    var mm = MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.instance_count = 1
    
    mm.set_instance_transform(0, Transform3D(Basis(Vector3(10, 20, 30), Vector3(11, 21, 31), Vector3(12, 22, 32)), Vector3(40, 41, 42)))
    mm.set_instance_color(0, Color(0.1, 0.2, 0.3, 0.4))
    
    var buf = mm.buffer
    print("BUF SIZE: ", buf.size())
    for i in range(buf.size()):
        print("BUF_VAL[" + str(i) + "]=" + str(buf[i]))
    
    quit()
