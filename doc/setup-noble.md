# Setup

### Remote Desktop

```sh
sudo apt update && sudo apt upgrade
sudo apt install ubuntu-desktop         # GUI
sudo apt install xrdp                   # Preparation for remote desktop access
```

### Add a user

```sh
sudo adduser <username>
sudo usermod -aG sudo <username>
```

### Hostname

```sh
hostnamectl
hostnamectl set-hostname new-hostname
```

### VSCode

- [Visual Studio Code on Linux](https://code.visualstudio.com/docs/setup/linux)
- [Download VSCode for mac arm64](https://code.visualstudio.com/docs/?dv=linuxarm64_deb)

```sh
echo "code code/add-microsoft-repo boolean true" | sudo debconf-set-selections
sudo apt install ~/Downloads/./<file>.deb
```

### Git

```sh
ssh-keygen -t ed25519 -C "tonik@n3xd.com"        ### not required for public users
git config --global core.editor "vim"
git config --global user.name "Toni Kaufmann"
git config --global user.email "tonik@n3xd.com"
git config --global pull.rebase true
```

### Docker

```sh
sudo snap install docker
sudo groupadd docker
sudo usermod -aG docker $USER
sudo reboot
```

### Miscellaneous

```sh
sudo apt install terminator
```

### Final touches

```sh
sudo timedatectl set-timezone Europe/Zurich
mkdir RosShared                                 # for using WebBots on the mac host
```

## Helpers

```sh
id -u  # returns UID
id -un # returns username
id -g  # returns GID
id -un # returns groupname
```
