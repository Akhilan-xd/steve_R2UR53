# steve_perception

Red-cube pose for **Steve**, from the pan camera (L515 on the pan-tilt tower) and the wrist camera (RealSense D405).

The workspace overview is in the [repository README](../../README.md). The pick and fetch nodes that consume this pose live in [steve_manipulation](../steve_manipulation/README.md).

## Layout

```text
src/steve_perception/
├── launch/cube_perception.launch.py
└── scripts/cube_perception.py
```

`cube_perception.py` finds a red blob in the color image, reads the range of that pixel from the matching depth image, shifts the point 2 cm along the camera ray (half of the 40 mm cube) so the pose is the cube center, and transforms it with TF.

A pose is published from whichever camera has a red blob with a finite depth. When the pan and wrist measurements agree within 6 cm, the closer wrist measurement is the one published. A missing depth image does not produce a pose: the stand height is never used as a stand-in for the range.

| Topic | Content |
| --- | --- |
| `/cube_pose` | Cube center in `map`, for the base |
| `/cube_pose_base` | Cube center in `base_link`, for the arm |
| `/cube_source` | `pan depth` or `wrist depth` |

Blobs are kept only when the range is between 0.25 m and 2.5 m and the point lies in front of the base, so red walls and furniture are dropped. The cube height is also required to fall between 0.55 m and 1.15 m.

## Run

`manipulation.launch.py` starts this node. To run it on its own, with the cameras already publishing:

```bash
ros2 launch steve_perception cube_perception.launch.py
```

## Maintainer

- **Akhilan Ashokan** — [akhilan.ashokan@smail.inf.h-brs.de](mailto:akhilan.ashokan@smail.inf.h-brs.de)
- Hochschule Bonn-Rhein-Sieg (H-BRS)
- GitHub: [Akhilan-xd/steve_R2UR53](https://github.com/Akhilan-xd/steve_R2UR53)

## License

Apache-2.0
