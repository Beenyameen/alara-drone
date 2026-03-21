using Godot;
using System;

public partial class ThrottleVisualiser : Control
{
	private int _throttle = 0;
	public int Throttle
	{
		get => _throttle;
		set
		{
			int clamped = Mathf.Clamp(value, 0, 100);
			if (_throttle != clamped)
			{
				_throttle = clamped;
				QueueRedraw();
			}
		}
	}

	private float _holdTimer = 0f;
	private const float InitialDelay = 0.3f;
	private const float RepeatRate = 0.02f;

	public override void _Ready()
	{
		if (CustomMinimumSize == Vector2.Zero)
		{
			CustomMinimumSize = new Vector2(40, 200);
		}
	}

	public override void _Process(double delta)
	{
		if (Input.IsActionJustPressed("pilot_throttle_up"))
		{
			Throttle += 1;
			_holdTimer = InitialDelay;
		}
		else if (Input.IsActionJustPressed("pilot_throttle_down"))
		{
			Throttle -= 1;
			_holdTimer = InitialDelay;
		}
		else if (Input.IsActionPressed("pilot_throttle_up"))
		{
			_holdTimer -= (float)delta;
			if (_holdTimer <= 0)
			{
				Throttle += 1;
				_holdTimer = RepeatRate;
			}
		}
		else if (Input.IsActionPressed("pilot_throttle_down"))
		{
			_holdTimer -= (float)delta;
			if (_holdTimer <= 0)
			{
				Throttle -= 1;
				_holdTimer = RepeatRate;
			}
		}
	}

	public override void _Draw()
	{
		Rect2 localRect = new Rect2(Vector2.Zero, Size);
		
		// Background
		DrawRect(localRect, new Color(0.15f, 0.15f, 0.15f, 1.0f));

		// Fill based on Throttle percentage
		float fillHeight = Size.Y * (Throttle / 100f);
		Rect2 fillRect = new Rect2(0, Size.Y - fillHeight, Size.X, fillHeight);
		DrawRect(fillRect, new Color(0.2f, 0.8f, 0.2f, 1.0f));

		// 10% increment lines
		for (int i = 1; i < 10; i++)
		{
			float y = Size.Y - (Size.Y * (i / 10f));
			DrawLine(new Vector2(0, y), new Vector2(Size.X, y), new Color(0, 0, 0, 0.6f), 2f);
		}

		// Border around the whole control
		DrawRect(localRect, Colors.Black, false, 2f);

		// Draw the number text centered
		Font font = ThemeDB.FallbackFont;
		int fontSize = 24;
		string text = Throttle.ToString();
		
		Vector2 textSize = font.GetStringSize(text, HorizontalAlignment.Left, -1, fontSize);
		float ascent = font.GetAscent(fontSize);
		
		Vector2 textPos = new Vector2(
			(Size.X - textSize.X) / 2f,
			(Size.Y - textSize.Y) / 2f + ascent
		);
		
		// Outline for text visibility against mixed background
		DrawStringOutline(font, textPos, text, HorizontalAlignment.Left, -1, fontSize, 2, Colors.Black);
		DrawString(font, textPos, text, HorizontalAlignment.Left, -1, fontSize, Colors.White);
	}
}
