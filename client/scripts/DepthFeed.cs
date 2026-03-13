using Godot;
using System;

public partial class DepthFeed : TextureRect
{
	public Image Image = Image.CreateEmpty(640, 480, false, Image.Format.Rgb8);
	public byte[] Rgb = new byte[640 * 480 * 3];

	public override void _Ready()
	{
		Texture = ImageTexture.CreateFromImage(Image);
	}

	public void UpdateFeed(byte[] data)
	{
		for (int i = 0; i < 640 * 480; i++)
		{
			byte b = (byte)Math.Min(((data[i * 2] | (data[i * 2 + 1] << 8)) * 255) / 4000, 255);
			Rgb[i * 3] = b;
			Rgb[i * 3 + 1] = b;
			Rgb[i * 3 + 2] = b;
		}
		Image.SetData(640, 480, false, Image.Format.Rgb8, Rgb);
		((ImageTexture)Texture).Update(Image);
	}
}
