# alara-drone

```bash
docker compose up --build
```




# Pi Setup Commands

echo "%sudo ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/99-nopasswd

sudo systemctl mask systemd-networkd-wait-online.service

sudo apt update && sudo apt full-upgrade -y && sudo apt autoremove -y




sudo sed -i 's/console=serial0,115200 //g' /boot/firmware/cmdline.txt

sudo sed -i 's/$/ usbcore.usbfs_memory_mb=128/' /boot/firmware/cmdline.txt

sudo apt install python3-gpiozero python3-lgpio python3-venv python3-dev python3-opencv cmake build-essential -y

sudo groupadd -f gpio

sudo usermod -aG dialout,gpio $USER




python3 -m venv venv

source venv/bin/activate

pip install pymavlink pyserial pyzmq pyorbbecsdk2 numpy




git clone https://github.com/orbbec/pyorbbecsdk.git

cd pyorbbecsdk

sudo bash ./scripts/env_setup/install_udev_rules.sh

sudo udevadm control --reload-rules && sudo udevadm trigger




# Pi Services

sudo nano /etc/systemd/system/ip.service

sudo nano /etc/systemd/system/camera.service

sudo nano /etc/systemd/system/geiger.service

sudo nano /etc/systemd/system/fc.service


sudo systemctl daemon-reload

sudo systemctl enable ip.service

sudo systemctl enable camera.service

sudo systemctl enable geiger.service

sudo systemctl enable fc.service
