using Godot;

public partial class Player : CharacterBody3D
{
	public Vector3 StartPos, StartRot, CamStartRot;
	public Camera3D Cam;

	public override void _Ready()
	{
		StartPos = Position;
		StartRot = Rotation;
		Cam = GetNode<Camera3D>("Camera3D");
		CamStartRot = Cam.Rotation;
		Input.MouseMode = Input.MouseModeEnum.Captured;
	}

	public override void _Input(InputEvent e)
	{
		if (e is InputEventMouseMotion m && Input.MouseMode == Input.MouseModeEnum.Captured)
		{
			RotateY(-m.Relative.X * 0.005f);
			Cam.RotateX(-m.Relative.Y * 0.005f);
			Cam.Rotation = new Vector3(Mathf.Clamp(Cam.Rotation.X, -1.5f, 1.5f), Cam.Rotation.Y, Cam.Rotation.Z);
		}
		if (e is InputEventMouseButton b && b.Pressed) Input.MouseMode = Input.MouseModeEnum.Captured;
		if (e.IsActionPressed("ui_cancel")) Input.MouseMode = Input.MouseModeEnum.Visible;
	}

	public override void _PhysicsProcess(double delta)
	{
		if (Input.IsActionJustPressed("world_reset"))
		{
			Position = StartPos;
			Rotation = StartRot;
			Cam.Rotation = CamStartRot;
			Velocity = Vector3.Zero;
			return;
		}
		Vector2 i = Input.GetVector("global_move_left", "global_move_right", "global_move_forward", "global_move_back");
		Vector3 d = (Transform.Basis * new Vector3(i.X, 0, i.Y)).Normalized();
		float s = 2.0f * (Input.IsActionPressed("world_boost") ? 1.5f : 1.0f);
		Vector3 v = Velocity;
		v.X = d.X * s;
		v.Z = d.Z * s;
		v.Y = 0;
		if (Input.IsActionPressed("world_ascend")) v.Y += s;
		if (Input.IsActionPressed("world_descend")) v.Y -= s;
		Velocity = v;
		MoveAndSlide();
	}
}
