from moviepy import VideoFileClip

clip = VideoFileClip("sph_simulation.mp4")
clip = clip.resized(width=480)  # keeps aspect ratio
clip.write_gif("sph_sim.gif", fps=12)
