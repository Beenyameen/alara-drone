using Godot;
using System;

public partial class Feed : TextureRect
{
	public enum Mode { Rgb, Depth }
	public Mode CurrentMode = Mode.Rgb;

	public Image Image = Image.CreateEmpty(640, 480, false, Image.Format.Rgb8);
	private byte[] _rgbBuffer = new byte[640 * 480 * 3];

	public override void _Ready() => Texture = ImageTexture.CreateFromImage(Image);

	public void UpdateFeedRgb(byte[] data)
	{
		Image.LoadJpgFromBuffer(data);
		((ImageTexture)Texture).Update(Image);
	}

	public void UpdateFeedDepth(byte[] data)
	{
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
