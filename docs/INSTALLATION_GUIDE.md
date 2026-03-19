# Installation Guide - Getting Ubuntu 24.04

This guide helps you set up Ubuntu 24.04 for the robotics labs. Once you have Ubuntu 24.04, return to the [main README](../README.md) to install Docker and run the labs.

---

## Option 1: WSL2 with Ubuntu 24.04 (Windows) - Recommended

[Watch follow-along Tutorial](https://www.youtube.com/watch?v=xPBFf_Tk0dw)

### Step 1: Enable WSL2

Open **PowerShell as Administrator**:

```powershell
wsl --install
```

Restart your computer, then:

```powershell
wsl --set-default-version 2
```

### Step 2: Install Ubuntu 24.04

1. Open **Microsoft Store**
2. Search "Ubuntu 24.04 LTS"
3. Install and launch
4. Create username and password

### Step 3: Install Docker Desktop

1. Download [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Install with "Use WSL 2 instead of Hyper-V"
3. Restart computer
4. Docker Desktop → Settings → Resources → WSL Integration
5. Enable Ubuntu-24.04

### Ready!

Return to the [main README](../README.md) Step 2 to clone the repository and continue setup.

**Tips:**
- Access Windows files: `/mnt/c/Users/YourName/`
- Access Ubuntu files from Windows: `\\wsl$\Ubuntu-24.04\home\yourname\`

---

## Option 2: Ubuntu 24.04 Dual Boot

### Step 1: Create Bootable USB

1. Download [Ubuntu 24.04 LTS](https://ubuntu.com/download/desktop)
2. Create bootable USB:
   - Windows: Use [Rufus](https://rufus.ie/)
   - macOS: Use [balenaEtcher](https://www.balena.io/etcher/)

### Step 2: Install Ubuntu

1. **Backup important data**
2. Boot from USB (F12/F2/DEL during startup)
3. Select "Install Ubuntu"
4. Choose "Install Ubuntu alongside" your current OS
5. Allocate 30-50 GB for Ubuntu
6. Create username and password
7. Restart

### Ready!

Boot into Ubuntu, then return to the [main README](../README.md) to install Docker and continue setup.

---

## Option 3: VirtualBox (Any OS)

[Watch follow-along Tutorial](https://www.youtube.com/watch?v=kSy3NX3Pe-c&list=PLNWNEEf8BvG6z60R4r9_wQ6Ekmqj-BmFr)

### Step 1: Install VirtualBox

1. Download [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
2. Download [Ubuntu 24.04 ISO](https://ubuntu.com/download/desktop)
3. Install VirtualBox

### Step 2: Create VM

1. VirtualBox → **New**
2. Settings:
   - Type: Linux, Ubuntu (64-bit)
   - RAM: 8192 MB
   - Hard Disk: 50 GB
   - Processors: 4 cores
   - Video Memory: 128 MB
3. Start VM and install Ubuntu from ISO

### Step 3: Install Guest Additions

After Ubuntu installation:

```bash
sudo apt-get update
sudo apt-get install -y build-essential dkms linux-headers-$(uname -r)
cd /media/$USER/VBox*
sudo ./VBoxLinuxAdditions.run
sudo reboot
```

## Setup Docker (skip if using WSL, it is supported out of the box)

### 1. Install Docker

```bash
# Install prerequisites
sudo apt-get update
sudo apt-get install -y git ca-certificates curl gnupg lsb-release

# Add Docker repository
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Verify installation
docker run hello-world
```

#### Nvidia GPU Support (Optional)

For better Gazebo performance with NVIDIA GPUs:

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```


### Ready!

Return to the [main README](../README.md) to install Docker and continue setup.

---

## Troubleshooting

### WSL2: Docker Not Working

PowerShell as Administrator:

```powershell
wsl --shutdown
```

Then restart Docker Desktop.

### WSL2: GUI Not Working

PowerShell as Administrator:

```powershell
wsl --update
```

### VirtualBox: Low Performance

- Allocate more CPU/RAM in VM settings
- Enable VT-x/AMD-V in BIOS
- Disable 3D acceleration if causing issues

---

## Additional Resources

- [GitHub Hello World Guide](https://docs.github.com/en/get-started/start-your-journey/hello-world)
- [WSL Documentation](https://docs.microsoft.com/en-us/windows/wsl/)
- [Docker for Robotics](https://articulatedrobotics.xyz/tutorials/docker/what-and-why)
