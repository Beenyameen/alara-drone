using Godot;
using System;

public partial class AttitudeVisualiser : Control
{
	private Vector2 _currentInput = Vector2.Zero;

	public Vector2 CurrentInput
	{
		get => _currentInput;
		set
		{
			// Only request a redraw if the input has actually changed to save performance
			if (_currentInput != value)
			{
				_currentInput = value;
				QueueRedraw();
			}
		}
	}

	public override void _Process(double delta)
	{
		// Poll input directly. This automatically normalizes the vector!
		// Using the actual bindings found in project.godot
		Vector2 inputVector = Input.GetVector("move_left", "move_right", "move_forward", "move_back");
		
		if (_currentInput != inputVector)
		{
			_currentInput = inputVector;
			QueueRedraw();
		}
	}

	public override void _Draw()
	{
		Vector2 center = Size / 2;
		// Keep the circle slightly inside the control bounds so the line thickness isn't clipped
		float radius = Mathf.Min(Size.X, Size.Y) / 2f - 4f; 

		// Draw the outer circle
		// Mathf.Tau is 2 * Pi (a full circle)
		DrawArc(center, radius, 0, Mathf.Tau, 64, Colors.White, 2f, true);

		// Draw lines crossing the diameter. 
		// You mentioned 3 lines offset by 45 degrees, which usually leaves an asymmetrical gap.
		// I've drawn 4 lines (0, 45, 90, 135 degrees) to make a perfectly symmetrical 8-slice pie.
		// If you only want 3, you can remove one of these angles!
		float[] angles = { 0, Mathf.Pi / 4, Mathf.Pi / 2, 3 * Mathf.Pi / 4 };
		foreach (float angle in angles)
		{
			Vector2 dir = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
			// Draw faint lines across the circle's diameter
			DrawLine(center - dir * radius, center + dir * radius, new Color(1, 1, 1, 0.3f), 1f, true);
		}

		// Draw the current input vector dot
		// Godot's Y axis points down, but Input.GetVector handles standard control mapping perfectly.
		Vector2 dotPos = center + (_currentInput * radius);
		
		// Draw a line connecting the center to the dot (optional, gives it a joystick look)
		DrawLine(center, dotPos, Colors.Red, 2f, true);
		
		// Draw the dot itself
		DrawCircle(dotPos, 6f, Colors.Red);
	}
}
