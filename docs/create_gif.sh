#!/bin/bash

vid whatsapp test.h264

typeset video="test-whatsapp.mp4"
typeset output="test.gif"

#ffmpeg -i /tmp/out.mkv -vf fps=25,scale=320:-1:flags=lanczos,palettegen /tmp/palette${thread_num}.png
ffmpeg -y -ss 0 -t 2 -i "$video" -vf fps=25,palettegen /tmp/palette.png

#ffmpeg -i /tmp/out.mkv -i /tmp/palette.png -filter_complex "fps=15,scale=400:-1:flags=lanczos[x];[x][1:v]paletteuse" "$output"
ffmpeg -y -ss 0 -t 1.5 -i "$video" -i /tmp/palette.png -filter_complex "scale=800:-1:flags=lanczos[x];[x][1:v]paletteuse" -r 3  "$output"

