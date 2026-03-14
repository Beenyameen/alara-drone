using Godot;
using System;

public partial class Drone : Node3D
{
	public Main M;
	public MultiMeshInstance3D Pts, Lines;

	public override void _Ready()
	{
		M = GetParent().GetParent() as Main;
		var m = new StandardMaterial3D { AlbedoColor = new Color(1, 0.2f, 0.2f), ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded };
		GetParent().CallDeferred("add_child", Pts = new MultiMeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, InstanceCount = 50000, Mesh = new SphereMesh { Radius = 0.02f, Height = 0.04f, Material = m } } });
		GetParent().CallDeferred("add_child", Lines = new MultiMeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, InstanceCount = 50000, Mesh = new CylinderMesh { TopRadius = 0.01f, BottomRadius = 0.01f, Height = 1f, Material = m } } });
	}

	public override void _Process(double delta)
	{
		var d = M.TrajectoryData;
		if (d == null || d.Length % 64 != 0 || d.Length == 0) return;
		
		int count = d.Length / 64;
		int numP = 0, numL = 0;
		Vector3 lastP = Vector3.Zero;

		for (int i = 0; i < count; i++)
		{
			int offset = i * 64;
			var b = new Basis(new Vector3(BitConverter.ToSingle(d, offset + 0), BitConverter.ToSingle(d, offset + 4), BitConverter.ToSingle(d, offset + 8)), new Vector3(BitConverter.ToSingle(d, offset + 16), BitConverter.ToSingle(d, offset + 20), BitConverter.ToSingle(d, offset + 24)), new Vector3(BitConverter.ToSingle(d, offset + 32), BitConverter.ToSingle(d, offset + 36), BitConverter.ToSingle(d, offset + 40)));
			var p = new Vector3(BitConverter.ToSingle(d, offset + 48), BitConverter.ToSingle(d, offset + 52), BitConverter.ToSingle(d, offset + 56));
			
			if (i == count - 1)
			{
				Transform = new Transform3D(b, p);
			}

			if (numP == 0 || p.DistanceTo(lastP) >= 0.05f)
			{
				if (numP >= 50000 || numL >= 50000) break;

				if (numP > 0)
				{
					var v = p - lastP;
					var y = v.Normalized();
					var x = Vector3.Up.Cross(y);
					if (x.LengthSquared() < 0.001f) x = Vector3.Right.Cross(y);
					Lines.Multimesh.SetInstanceTransform(numL++, new Transform3D(new Basis(x.Normalized(), v, x.Normalized().Cross(y).Normalized()), (lastP + p) / 2f));
				}
				Pts.Multimesh.SetInstanceTransform(numP++, new Transform3D(Basis.Identity, lastP = p));
			}
		}
		
		Lines.Multimesh.VisibleInstanceCount = numL;
		Pts.Multimesh.VisibleInstanceCount = numP;
	}
}
