# V2AIX replay demo

Minimal compose setup to replay V2AIX ROS2 bags, including the ETSI ITS conversion node, and visualize messages in RViz.

## Quick start

Copy and adjust the env file, and run the composition:

```bash
docker compose --env-file .ros2-demo-a44.env -f docker-compose.yml
```

## Configuration

The `.env`/`.ros2-demo-a44.env` file controls:
- `BAG_DIR` (host path containing the bag)
- `BAG_PATH` (container path to the bag)
- `RVIZ_LAT` and `RVIZ_LON` (Aerial Map Display position)

If RViz does not show up, ensure X11 permissions are set (e.g. `xhost +local:`) and `DISPLAY` is available.
