extends SceneTree

func _init():
    var mm = MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.instance_count = 1
    
    mm.set_instance_transform(0, Transform3D(Basis(Vector3(1, 2, 3), Vector3(4, 5, 6), Vector3(7, 8, 9)), Vector3(10, 20, 30)))
    mm.set_instance_color(0, Color(0.1, 0.2, 0.3, 0.4))
    
    # Force an update of the buffer by getting it. Wait, `buffer` might be cached.
    var buf = mm.buffer
    for i in range(buf.size()):
        print("BUF_VAL[" + str(i) + "]=" + str(buf[i]))
        
    quit()
