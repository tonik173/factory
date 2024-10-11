Setup



```sh
sudo apt update && sudo apt upgrade
sudo apt install ubuntu-desktop xrdp -y     # Preparation for remote desktop access
sudo adduser <username>
sudo usermod -aG sudo <username>
```





```sh
sudo timedatectl set-timezone Europe/Zurich
ssh-keygen -t ed25519 -C "tonik@n3xd.com"


git config --global core.editor "vim"
git config --global user.name "Toni Kaufmann"
git config --global user.email "tonik@n3xd.com"
git config --global pull.rebase true
```

