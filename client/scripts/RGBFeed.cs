using Godot;

public partial class RGBFeed : TextureRect
{
	public Image Image = Image.CreateEmpty(640, 480, false, Image.Format.Rgb8);

	public override void _Ready()
	{
		Texture = ImageTexture.CreateFromImage(Image);
	}

	public void UpdateFeed(byte[] data)
	{
		Image.LoadJpgFromBuffer(data);
		((ImageTexture)Texture).Update(Image);
	}
}
