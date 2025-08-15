
Structure: 

```bash
xshan/
├─ deploy/
│  ├─ compose/                     # docker-compose stacks (dev/prod/profiles)
│  │  ├─ docker-compose.yml        # master compose (profiles: hotpath, infra, nav2, slam, ai)
│  │  └─ .env.example
│  └─ k8s/                         # (future) k8s manifests, if you go there later
│
├─ interfaces/                     # STABLE CONTRACTS live here
│  ├─ interfaces/                  # ROS 2 msg/srv packages (colcon workspace style)
│  │  ├─ CMakeLists.txt
│  │  ├─ package.xml
│  │  └─ msg/
│  │     ├─ Detection2D.msg        # (bbox, score, class_id, track_id optional)
│  │     ├─ Detection2DArray.msg
│  │     ├─ Track2D.msg            # (id, bbox, velocity, age, confidence)
│  │     └─ Track2DArray.msg
│  └─ api/                         # non-ROS contracts for services (gRPC/OpenAPI)
│     ├─ vlm.proto
│     ├─ stt.proto
│     └─ tts.proto
│
├─ config/                         # System-level config (hardware-agnostic)
│  ├─ system/
│  │  ├─ cyclone_dds.xml           # CycloneDDS tuning
│  │  └─ qos_profiles.yaml         # QoS profiles for topics
│  ├─ hotpath/
│  │  ├─ detector_rgb.yaml         # model/runtime params
│  │  ├─ fusion.yaml
│  │  └─ tracker.yaml
│  └─ camera/
│     ├─ uvc_rgb_1280x720.yaml     # camera_info for Arducam UVC
│     └─ csi_imx219_1920x1080.yaml
│
├─ models/                         # Source/managed models (tracked in Git or DVC)
│  ├─ yolo_rgb_v1/
│  │  ├─ model.onnx
│  │  ├─ calibration.cache
│  │  └─ model.yaml
│  └─ README.md
│
├─ models_cache/                   # Runtime artifacts (TensorRT engines, etc.) (gitignored)
│  └─ yolo_rgb_v1/
│     └─ model.trt
│
├─ hotpath/                        # ONE PROCESS: ingest → preproc → detect → track → fuse
│  ├─ Dockerfile
│  ├─ entrypoint.sh
│  ├─ healthcheck.sh
│  ├─ launch/
│  │  └─ hotpath.launch.py         # ComposableNodeContainer wiring components below
│  ├─ src/
│  │  ├─ image_source/             # camera sources as components (UVC now; more later)
│  │  │  ├─ gscam_source_component.cpp
│  │  │  └─ params.yaml            # (caps, fps, pipeline knobs)
│  │  ├─ preproc/                  # GPU resize/normalize (TensorRT-friendly)
│  │  │  └─ preproc_component.cpp
│  │  ├─ detector/
│  │  │  ├─ tensorrt_component.cpp # loads engines from models_cache/
│  │  │  └─ labels.txt
│  │  ├─ tracker/
│  │  │  └─ bytetrack_component.cpp
│  │  └─ fusion/
│  │     └─ mid_fusion_component.cpp
│  ├─ include/                     # headers for components
│  ├─ CMakeLists.txt
│  └─ package.xml
│
├─ sensors/                        # Standalone sensor containers (optional / transitional)
│  └─ uvc_rgb/
│     ├─ Dockerfile
│     ├─ start.sh                  # fast NVMM pipeline (v4l2src→nvjpegdec/nvv4l2decoder→nvvidconv)
│     ├─ healthcheck.sh
│     └─ config/
│        └─ uvc_rgb_1280x720.yaml
│
├─ infra/                          # Telemetry/UI gateways (Foxglove, rosbridge, web video)
│  ├─ Dockerfile
│  ├─ start.sh                     # foxglove_bridge (ws://0.0.0.0:${FOXGLOVE_PORT})
│  └─ README.md
│
├─ nav2/                           # Navigation stack container (decoupled)
│  ├─ Dockerfile
│  ├─ start.sh
│  ├─ behavior_trees/
│  └─ config/
│     └─ nav2.yaml
│
├─ slam/                           # SLAM toolbox / Cartographer
│  ├─ Dockerfile
│  ├─ start.sh
│  └─ config/
│     └─ slam.yaml
│
├─ localization/                   # robot_localization (ekf/ukf)
│  ├─ Dockerfile
│  ├─ start.sh
│  └─ config/
│     └─ ekf.yaml
│
├─ services/                       # Non-ROS microservices (gRPC/HTTP) – easy to scale/isolate
│  ├─ vlm/
│  │  ├─ Dockerfile
│  │  └─ server.py
│  ├─ stt/
│  │  ├─ Dockerfile
│  │  └─ server.py
│  └─ tts/
│     ├─ Dockerfile
│     └─ server.py
│
├─ viz/
│  └─ qt/
│     ├─ Dockerfile
│     └─ app/                      # optional Qt UI (local/remote)
│
├─ tools/                          # Dev & ops tools (profiling, calibration, TRT build)
│  ├─ build_trt_engine.py
│  ├─ profile_pipeline.sh
│  └─ calibrate_camera.sh
│
├─ .gitignore
├─ .dockerignore
└─ README.md
```