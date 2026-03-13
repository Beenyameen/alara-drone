using Godot;
using System;

public partial class Drone : Node3D
{
	public Main M;
	public MultiMeshInstance3D Pts, Lines;
	public Vector3 LastP;
	public int NumP, NumL;

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
		if (d == null || d.Length != 64) return;
		var b = new Basis(new Vector3(BitConverter.ToSingle(d, 0), BitConverter.ToSingle(d, 4), BitConverter.ToSingle(d, 8)), new Vector3(BitConverter.ToSingle(d, 16), BitConverter.ToSingle(d, 20), BitConverter.ToSingle(d, 24)), new Vector3(BitConverter.ToSingle(d, 32), BitConverter.ToSingle(d, 36), BitConverter.ToSingle(d, 40)));
		var e = b.GetEuler(); e.X = -e.X;
		var p = new Vector3(BitConverter.ToSingle(d, 48), BitConverter.ToSingle(d, 52), BitConverter.ToSingle(d, 56));
		Transform = new Transform3D(Basis.FromEuler(e), p);
		if (NumP == 0 || p.DistanceTo(LastP) >= 0.05f)
		{
			if (NumP > 0)
			{
				var v = p - LastP;
				var y = v.Normalized();
				var x = Vector3.Up.Cross(y);
				if (x.LengthSquared() < 0.001f) x = Vector3.Right.Cross(y);
				Lines.Multimesh.SetInstanceTransform(NumL++, new Transform3D(new Basis(x.Normalized(), v, x.Normalized().Cross(y).Normalized()), (LastP + p) / 2f));
				Lines.Multimesh.VisibleInstanceCount = NumL;
			}
			Pts.Multimesh.SetInstanceTransform(NumP++, new Transform3D(Basis.Identity, LastP = p));
			Pts.Multimesh.VisibleInstanceCount = NumP;
		}
	}
}
