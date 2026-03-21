using Godot;
using System;

public partial class Drone : Node3D
{
	public Main M;
	public MultiMeshInstance3D Pts, Lines, Cones;
	public MeshInstance3D FirstPt;

	public override void _Ready()
	{
		Node n = this;
		while (n != null && !(n is Main)) n = n.GetParent();
		M = n as Main;
		var m = new StandardMaterial3D { AlbedoColor = new Color(1, 0.2f, 0.2f), ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded };
		var greenMat = new StandardMaterial3D { AlbedoColor = new Color(0, 1, 0), EmissionEnabled = true, Emission = new Color(0, 1, 0), EmissionEnergyMultiplier = 4.0f };
		GetParent().CallDeferred("add_child", Pts = new MultiMeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, InstanceCount = 50000, Mesh = new SphereMesh { Radius = 0.02f, Height = 0.04f, Material = m } } });
		GetParent().CallDeferred("add_child", Lines = new MultiMeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, InstanceCount = 50000, Mesh = new CylinderMesh { TopRadius = 0.01f, BottomRadius = 0.01f, Height = 1f, Material = m } } });
		GetParent().CallDeferred("add_child", Cones = new MultiMeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, InstanceCount = 50000, Mesh = new CylinderMesh { TopRadius = 0.0f, BottomRadius = 0.02f, Height = 0.06f, Material = m } } });
		GetParent().CallDeferred("add_child", FirstPt = new MeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Mesh = new SphereMesh { Radius = 0.025f, Height = 0.05f, Material = greenMat }, Visible = false });

		var pyrMat = new StandardMaterial3D { AlbedoColor = new Color(0, 1, 1), ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded };
		var im = new ImmediateMesh();
		var pyrMi = new MeshInstance3D { CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Mesh = im, MaterialOverride = pyrMat };
		AddChild(pyrMi);

		im.SurfaceBegin(Mesh.PrimitiveType.Lines);
		var apex = new Vector3(0, 0, 0);
		var c1 = new Vector3(0.1f, 0.075f, -0.2f);
		var c2 = new Vector3(-0.1f, 0.075f, -0.2f);
		var c3 = new Vector3(-0.1f, -0.075f, -0.2f);
		var c4 = new Vector3(0.1f, -0.075f, -0.2f);

		im.SurfaceAddVertex(apex); im.SurfaceAddVertex(c1);
		im.SurfaceAddVertex(apex); im.SurfaceAddVertex(c2);
		im.SurfaceAddVertex(apex); im.SurfaceAddVertex(c3);
		im.SurfaceAddVertex(apex); im.SurfaceAddVertex(c4);

		im.SurfaceAddVertex(c1); im.SurfaceAddVertex(c2);
		im.SurfaceAddVertex(c2); im.SurfaceAddVertex(c3);
		im.SurfaceAddVertex(c3); im.SurfaceAddVertex(c4);
		im.SurfaceAddVertex(c4); im.SurfaceAddVertex(c1);
		im.SurfaceEnd();
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
			var raw_b = new Basis(new Vector3(BitConverter.ToSingle(d, offset + 0), BitConverter.ToSingle(d, offset + 4), BitConverter.ToSingle(d, offset + 8)), new Vector3(BitConverter.ToSingle(d, offset + 16), BitConverter.ToSingle(d, offset + 20), BitConverter.ToSingle(d, offset + 24)), new Vector3(BitConverter.ToSingle(d, offset + 32), BitConverter.ToSingle(d, offset + 36), BitConverter.ToSingle(d, offset + 40)));
			var raw_p = new Vector3(BitConverter.ToSingle(d, offset + 48), BitConverter.ToSingle(d, offset + 52), BitConverter.ToSingle(d, offset + 56));
			
			var t_cw = new Transform3D(raw_b, raw_p);
			var t_wc = t_cw.Inverse();
			var b_wc = t_wc.Basis;

			var b = new Basis(
				new Vector3(b_wc.X.X, -b_wc.X.Y, -b_wc.X.Z),
				new Vector3(-b_wc.Y.X, b_wc.Y.Y, b_wc.Y.Z),
				new Vector3(-b_wc.Z.X, b_wc.Z.Y, b_wc.Z.Z)
			);
			var p = new Vector3(t_wc.Origin.X, -t_wc.Origin.Y, -t_wc.Origin.Z);
			
			if (i == count - 1)
			{
				Transform = new Transform3D(b, p);
			}

			if (numP == 0 || p.DistanceTo(lastP) >= 0.10f)
			{
				if (numP >= 50000 || numL >= 50000) break;

				if (numP > 0)
				{
					var v = p - lastP;
					var y = v.Normalized();
					var x = Vector3.Up.Cross(y);
					if (x.LengthSquared() < 0.001f) x = Vector3.Right.Cross(y);
					Lines.Multimesh.SetInstanceTransform(numL, new Transform3D(new Basis(x.Normalized(), v, x.Normalized().Cross(y).Normalized()), (lastP + p) / 2f));
					Cones.Multimesh.SetInstanceTransform(numL++, new Transform3D(new Basis(x.Normalized(), y, x.Normalized().Cross(y).Normalized()), (lastP + p) / 2f));
				}
				else
				{
					FirstPt.Transform = new Transform3D(Basis.Identity, p);
					FirstPt.Visible = true;
				}
				Pts.Multimesh.SetInstanceTransform(numP++, new Transform3D(Basis.Identity, lastP = p));
			}
		}
		
		Lines.Multimesh.VisibleInstanceCount = numL;
		Cones.Multimesh.VisibleInstanceCount = numL;
		Pts.Multimesh.VisibleInstanceCount = numP;
	}
}
