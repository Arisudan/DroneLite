# Syncing this folder through GitHub (https://github.com/Arisudan/DroneLite)

This repository holds the **code** of the DroneForge project (2026-11-24 state). It does not hold: the model weights (`gazebo/vision/models/`, 1.4 GB), the third-party Gazebo house
(`gazebo/third_party/small_house_src`), flight recordings and FC logs (`docs/`), `archive/`, `reference/`, Claude memory notes, FC config dumps (they contain the receiver UID) and
everything ELRS-binding related (the binding phrase / UID are `<BINDING_PHRASE>` / `<BINDING_UID>` placeholders in README.md; your real values stay local).

On a machine that already has a full DroneForge folder: clone the repository next to it and copy the code over (nothing is deleted):

    git clone https://github.com/Arisudan/DroneLite ~/DroneLite && cd ~/DroneLite && git checkout <branch>      # later: git pull
    rsync -a --exclude='/README.md' --exclude='.git*' ~/DroneLite/DroneForge/ ~/Music/DroneForge/

`--exclude='/README.md'` keeps your local README.md (with the real binding phrase). Then run `./setup_new_laptop.sh` once in the DroneForge folder (it also re-points old absolute paths).
