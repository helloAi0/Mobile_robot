# Media for the front page

The three files here are placeholders. Overwrite them, keep the names, commit, push. The README picks them up.

| file | what to put there |
|---|---|
| `demo.gif` | 15 to 30 s screen recording: robot driving in Gazebo with RViz next to it (map building, lidar, camera cloud) |
| `gazebo.png` | one screenshot of the Gazebo window (the house, the robot, people walking) |
| `rviz.png` | one screenshot of RViz (map, magenta lidar, point cloud, orange detections) |

## Recording to GIF (Linux / WSL, ffmpeg)

Record the screen with any tool (Windows: Win+G Game Bar, or OBS) and save as `demo.mp4`, then:

```bash
ffmpeg -i demo.mp4 -vf "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer" -loop 0 docs/media/demo.gif
```

Keep the GIF under about 10 MB (GitHub shows it inline; bigger loads slowly): shorten the clip or lower `fps` / `scale` if needed.
For LinkedIn upload the original `demo.mp4`, not the GIF.

Suggested clip (also in [../DEMO.md](../DEMO.md)): drive forward with the keyboard, stop in front of a wall (the safety layer), press `p` for the autopilot, show the map growing in RViz.
