using System.IO;
using Godot;

public partial class TestMultiMesh : SceneTree
{
    public override void _Initialize()
    {
        var mm = new MultiMesh();
        mm.TransformFormat = MultiMesh.TransformFormatEnum.Transform3D;
        mm.UseColors = true;
        mm.InstanceCount = 1;
        
        mm.SetInstanceTransform(0, new Transform3D(Basis.Identity, new Vector3(10, 20, 30)));
        mm.SetInstanceColor(0, new Color(1, 2, 3, 4));
        
        float[] buffer = mm.Buffer;
        
        using (var writer = new StreamWriter("multimesh_layout.txt"))
        {
            for(int i = 0; i < buffer.Length; i++) {
                writer.WriteLine($"buf[{i}] = {buffer[i]}");
            }
        }
        Quit();
    }
}
