sudo systemctl restart xyz_autostart.service
sudo systemctl status xyz_autostart.service
/usr/bin/supervisord -c /home/xyz/xyz_app/central_hub/supervisor/supervisord.conf


while the xyz_autostart.service, we need to reload units
```
systemctl daemon-reload
```

sudo supervisorctl update
or 
sudo supervisorctl reload