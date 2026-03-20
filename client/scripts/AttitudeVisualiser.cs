using Godot;
using System;

public partial class AttitudeVisualiser : Control
{
	private Main _main;
	private Vector3 _imu;

	public override void _Ready()
	{
		Node n = this;
		while (n != null && !(n is Main)) n = n.GetParent();
		_main = n as Main;
	}

	public override void _Process(double delta)
	{
		if (_main != null && _main.ImuData != _imu)
		{
			_imu = _main.ImuData;
			QueueRedraw();
		}
	}

	public override void _Draw()
	{
		Vector2 center = Size / 2;
		float r = Mathf.Min(Size.X, Size.Y) / 2f - 4f; 
		DrawArc(center, r, 0, Mathf.Tau, 64, Colors.White, 2f, true);

		Vector2 dir = new Vector2(Mathf.Cos(-_imu.X), Mathf.Sin(-_imu.X));
		Vector2 perp = new Vector2(-dir.Y, dir.X);
		float po = _imu.Y * r; 

		for (int i = -4; i <= 4; i++)
		{
			float off = po + (i * 0.25f * r);
			float d = Mathf.Abs(off);
			if (d >= r) continue;
			
			float chord = Mathf.Sqrt(r * r - d * d);
			float w = i == 0 ? chord : Mathf.Min(chord, i % 2 == 0 ? r * 0.4f : r * 0.2f);
			Color c = i == 0 ? Colors.Green : new Color(1, 1, 1, 0.5f);
			float t = i == 0 ? 2f : 1.5f;
			
			Vector2 lc = center + perp * off;
			DrawLine(lc - dir * w, lc + dir * w, c, t, true);
		}

		DrawLine(center - new Vector2(15, 0), center - new Vector2(5, 0), Colors.Red, 2f);
		DrawLine(center + new Vector2(5, 0), center + new Vector2(15, 0), Colors.Red, 2f);
		DrawLine(center, center + new Vector2(0, -5), Colors.Red, 2f);
	}
}
