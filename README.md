# DroneLite

DroneForge: an indoor drone built around a Betaflight flight controller (Matek H743, custom height / position hold firmware), an ELRS radio link driven by a PC,
a DJI O4 camera through CosmoStreamer, and a PC ground station (GCS) with depth, detection, visual odometry and RTAB-Map mapping. Simulated in Gazebo Harmonic + ROS 2 Jazzy.

| Folder | What it is |
|---|---|
| `DroneForge/` | The live project (2026-11-24). Start with `DroneForge/HANDOFF.md`, then `DroneForge/README.md`. |
| `New/` | Files copied over from the previous PC: an older copy of the project, the pre-reorganisation tree (2026-11-02) and dated backups. |

## Not in this repository
- **Files over 100 MB** (GitHub's limit): the Depth Anything 3 weights `gazebo/vision/models/nimbus/da3_metric.weights.safetensors` (1.3 GB) in both copies, and the packaged
  `.tar` backups. They stay on the development PC.
- **Radio binding data**: the ELRS binding phrase and binding UID are replaced with `<BINDING_PHRASE>` / `<BINDING_UID>` (all-zero UID in the TX settings JSON files), and the
  built TX firmware images, which contain them, are left out. Rebuild with `tools_scripts/tx_build/tx_build_patch_flash_elrs.sh <work> <port> <your phrase>`.
- Nested git checkouts (their `.git` folders), Python caches and build output.
