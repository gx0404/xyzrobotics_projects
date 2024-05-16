sudo systemctl stop xyz_autostart.service
sudo systemctl disable xyz_autostart.service
sudo dpkg -r xyz-supervisor
sudo dpkg -r xyz-supervisor-test
rm -rf /home/xyz/xyz_app/central_hub
