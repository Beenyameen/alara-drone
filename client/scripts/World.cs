using Godot;

public partial class World : Node3D
{
	public MultiMeshInstance3D Mm;
	public Player P;
	public int LastX = int.MinValue, LastZ = int.MinValue;

	public override void _Ready()
	{
		var sb = new StaticBody3D();
		sb.AddChild(new CollisionShape3D { Shape = new WorldBoundaryShape3D() });
		AddChild(sb);
		var originMarker = new CsgPolygon3D {
			Polygon = new Vector2[] { new Vector2(-0.5f, -0.5f), new Vector2(0.5f, -0.5f), new Vector2(0.5f, 0.5f), new Vector2(-0.5f, 0.5f), new Vector2(-0.5f, -0.5f), new Vector2(-0.4f, -0.4f), new Vector2(-0.4f, 0.4f), new Vector2(0.4f, 0.4f), new Vector2(0.4f, -0.4f), new Vector2(-0.4f, -0.4f) },
			Mode = CsgPolygon3D.ModeEnum.Depth,
			Depth = 0.001f,
			MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0, 1, 0), EmissionEnabled = true, Emission = new Color(0, 1, 0), EmissionEnergyMultiplier = 4.0f }
		};
		originMarker.Rotation = new Vector3(-Mathf.Pi / 2, 0, 0);
		originMarker.Position = new Vector3(0, 0.01f, 0);
		AddChild(originMarker);
		Mm = new MultiMeshInstance3D {
			Multimesh = new MultiMesh {
				TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
				UseColors = true,
				InstanceCount = 31500,
				Mesh = new PlaneMesh { Size = new Vector2(1, 1), Material = new StandardMaterial3D { VertexColorUseAsAlbedo = true } }
			}
		};
		P = GetNode<Player>("Player");
		AddChild(Mm);
	}

	public override void _Process(double delta)
	{
		int px = Mathf.FloorToInt(P.Position.X), pz = Mathf.FloorToInt(P.Position.Z);
		if (px == LastX && pz == LastZ) return;
		LastX = px; LastZ = pz;
		int i = 0;
		for (int x = -100; x <= 100; x++)
		for (int z = -100; z <= 100; z++)
			if (x * x + z * z <= 10000)
			{
				int wx = px + x, wz = pz + z;
				Mm.Multimesh.SetInstanceTransform(i, new Transform3D(Basis.Identity, new Vector3(wx + 0.5f, 0, wz + 0.5f)));
				Mm.Multimesh.SetInstanceColor(i++, (wx + wz) % 2 == 0 ? new Color(0.2f, 0.2f, 0.2f) : new Color(0.8f, 0.8f, 0.8f));
			}
		Mm.Multimesh.VisibleInstanceCount = i;
	}
}
