! mkdir -p res

set term pngcairo size 960, 960

set size ratio -1

set xr [-0.5:2.0]
set yr [-0.5:3.0]

unset key; unset tics

do for [i=1:179] {
	set output sprintf('res/image%05d.png', i)

	pl 'walls.dat' w lines lw 2 lc rgb '#000000', \
	   sprintf('data/output%05d.dat', i) w linesp pt 7 ps 0.2 lw 2 lc rgb '#0060ad'


}
unset output

# ! ffmpeg -framerate 30 -i res/image%05d.png -c:v libx264 video.mp4