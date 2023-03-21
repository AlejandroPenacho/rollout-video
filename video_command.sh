ffmpeg -framerate 60 -i frames/%05d.png -vf scale=1906x1044,crop=1200:1044:431:0 output.mp4
