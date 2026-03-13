using Godot;

public partial class Player : CharacterBody3D
{
	public override void _PhysicsProcess(double delta)
	{
		Vector3 v = Velocity;
		if (!IsOnFloor()) v += GetGravity() * (float)delta;
		if (Input.IsActionJustPressed("ascend") && IsOnFloor()) v.Y = 4.5f;

		Vector2 i = Input.GetVector("move_left", "move_right", "move_forward", "move_back");
		Vector3 d = (Transform.Basis * new Vector3(i.X, 0, i.Y)).Normalized();
		
		v.X = d != Vector3.Zero ? d.X * 5.0f : Mathf.MoveToward(Velocity.X, 0, 5.0f);
		v.Z = d != Vector3.Zero ? d.Z * 5.0f : Mathf.MoveToward(Velocity.Z, 0, 5.0f);

		Velocity = v;
		MoveAndSlide();
	}
}
