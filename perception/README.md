# Detector 

From `xshan/deploy/compose`:

### Build (detector only, fresh)

```bash
docker compose build detector --no-cache --pull
```

### Start / Restart / Stop (detector only)

```bash
# start
docker compose up -d detector

# restart
docker compose restart detector

# stop
docker compose stop detector

# remove container (keep image)
docker compose rm -f detector
```

### Watch logs

```bash
docker logs -f xshan-detector
```

### Rebuild + redeploy in one go (after edits)

```bash
docker compose build detector --no-cache --pull \
&& docker compose up -d --force-recreate detector
```

---
docker compose build detector --no-cache --pull
ocker compose up -d --force-recreate detector
## Quick verification (no ROS on host needed)

Run these **inside a container** that has ROS (detector or infra).

### See topics (use infra or detector)

```bash
# either one:
docker exec -it xshan-infra bash -lc 'ros2 topic list'
# or:
docker exec -it xshan-detector bash -lc 'ros2 topic list'
```

You should see:

```
/sensors/uvc/image_raw
/perception/image_annotated
/perception/detections
```

### See one detection message

```bash
docker exec -it xshan-infra bash -lc \
'ros2 topic echo -n 1 /perception/detections vision_msgs/msg/Detection2DArray'
```

### See one annotated image (proves the image pipeline in detector)

```bash
docker exec -it xshan-infra bash -lc \
"ros2 topic echo -n 1 /perception/image_annotated sensor_msgs/msg/Image \
 --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 1"
```


```bash
 docker exec -it xshan-detector bash -lc \
'source /opt/ros/humble/setup.bash && ros2 topic info -v /perception/image_annotated'
```
