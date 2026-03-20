using Godot;
using System;

public partial class HeadingVisualiser : Control
{
	private Main _main;
	private float _yaw;

	public override void _Ready()
	{
		Node n = this;
		while (n != null && !(n is Main)) n = n.GetParent();
		_main = n as Main;
	}

	public override void _Process(double delta)
	{
		if (_main != null && _main.ImuData.Z != _yaw)
		{
			_yaw = _main.ImuData.Z;
			QueueRedraw();
		}
	}

	public override void _Draw()
	{
		Vector2 center = Size / 2;
		float r = Mathf.Min(Size.X, Size.Y) / 2f - 4f; 
		
		DrawArc(center, r, 0, Mathf.Tau, 64, Colors.White, 2f, true);

		Font font = ThemeDB.FallbackFont;
		int fontSize = 14;
		float ascent = font.GetAscent(fontSize);

		// Fixed Compass Marks
		for (int i = 0; i < 360; i += 15)
		{
			float angleRad = Mathf.DegToRad(i) - Mathf.Pi / 2f;
			Vector2 dir = new Vector2(Mathf.Cos(angleRad), Mathf.Sin(angleRad));
			
			if (i % 90 == 0)
			{
				float innerR = r - 12;
				DrawLine(center + dir * innerR, center + dir * r, Colors.White, 2f, true);
				
				string label = i == 0 ? "N" : i == 90 ? "E" : i == 180 ? "S" : "W";
				Vector2 textSize = font.GetStringSize(label, HorizontalAlignment.Left, -1, fontSize);
				
				Vector2 textPos = center + dir * (innerR - 10) - new Vector2(textSize.X / 2f, -ascent / 2f);
				DrawString(font, textPos, label, HorizontalAlignment.Left, -1, fontSize, Colors.White);
			}
			else
			{
				float innerR = r - 6;
				DrawLine(center + dir * innerR, center + dir * r, new Color(1, 1, 1, 0.7f), 1f, true);
			}
		}

		// Rotating Dial
		float deg = (_yaw * 180f / Mathf.Pi) % 360f;
		if (deg < 0) deg += 360f;

		float headingRad = _yaw - Mathf.Pi / 2f;
		Vector2 dialDir = new Vector2(Mathf.Cos(headingRad), Mathf.Sin(headingRad));

		float dialLength = r - 30;
		DrawLine(center, center + dialDir * dialLength, Colors.Red, 2f, true);
		DrawCircle(center, 4f, Colors.Red);

		string yText = $"{deg,3:F0}°";
		Vector2 ySize = font.GetStringSize(yText, HorizontalAlignment.Left, -1, fontSize);
		
		Vector2 yPos = center + dialDir * (dialLength + 14) - new Vector2(ySize.X / 2f, -ascent / 2f);
		DrawStringOutline(font, yPos, yText, HorizontalAlignment.Left, -1, fontSize, 2, Colors.Black);
		DrawString(font, yPos, yText, HorizontalAlignment.Left, -1, fontSize, Colors.Red);
	}
}
