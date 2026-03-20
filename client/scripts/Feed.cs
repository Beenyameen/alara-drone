using Godot;
using System;

public partial class Feed : TextureRect
{
	public enum Mode { Rgb, Depth }
	public Mode CurrentMode = Mode.Rgb;

	public Image Image = Image.CreateEmpty(640, 480, false, Image.Format.Rgb8);
	private byte[] _rgbBuffer = new byte[640 * 480 * 3];
	public int FramesReceived = 0;

	public float R, P, Y, T;

	public override void _Ready() => Texture = ImageTexture.CreateFromImage(Image);

	public override void _Process(double delta)
	{
		if (Input.IsActionPressed("ascend")) T = Mathf.Clamp(T + (float)delta * 0.5f, 0, 1);
		if (Input.IsActionPressed("descend")) T = Mathf.Clamp(T - (float)delta * 0.5f, 0, 1);
		if (Input.IsKeyPressed(Key.Q)) Y = Mathf.Clamp(Y - (float)delta * 2f, -1, 1);
		else if (Input.IsKeyPressed(Key.E)) Y = Mathf.Clamp(Y + (float)delta * 2f, -1, 1);
		else Y = Mathf.MoveToward(Y, 0, (float)delta * 2f);
		var i = Input.GetVector("move_left", "move_right", "move_forward", "move_back");
		R = i.X; P = i.Y;
	}

	public void UpdateFeedRgb(byte[] data)
	{
		FramesReceived++;
		Image.LoadJpgFromBuffer(data);
		((ImageTexture)Texture).Update(Image);
	}

	public void UpdateFeedDepth(byte[] data)
	{
		FramesReceived++;
		for (int i = 0; i < 640 * 480; i++)
		{
			byte b = (byte)Math.Min(((data[i * 2] | (data[i * 2 + 1] << 8)) * 255) / 4000, 255);
			_rgbBuffer[i * 3] = b;
			_rgbBuffer[i * 3 + 1] = b;
			_rgbBuffer[i * 3 + 2] = b;
		}
		Image.SetData(640, 480, false, Image.Format.Rgb8, _rgbBuffer);
		((ImageTexture)Texture).Update(Image);
	}
}
