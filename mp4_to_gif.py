from moviepy import VideoFileClip

clip = VideoFileClip("sph_simulation_white_bg.mp4")
clip = clip.resized(width=480)  # keeps aspect ratio
clip.write_gif("sph_sim_white_bg.gif", fps=12)
