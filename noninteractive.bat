"bin/ycolorgrade" --image tests/greg_zaal_artist_workshop.hdr --exposure 0 --output out/greg_zaal_artist_workshop_01.jpg
"bin/ycolorgrade" --image tests/greg_zaal_artist_workshop.hdr --exposure 1 --filmic --contrast 0.75 --saturation 0.75 --output out/greg_zaal_artist_workshop_02.jpg
"bin/ycolorgrade" --image tests/greg_zaal_artist_workshop.hdr --exposure 0.8 --contrast 0.6 --saturation 0.5 --grain 0.5 --output out/greg_zaal_artist_workshop_03.jpg

"bin/ycolorgrade" --image tests/toa_heftiba_people.jpg --exposure -1 --filmic --contrast 0.75 --saturation 0.3 --vignette 0.4 --output out/toa_heftiba_people_01.jpg
"bin/ycolorgrade" --image tests/toa_heftiba_people.jpg --exposure -0.5 --contrast 0.75 --saturation 0 --output out/toa_heftiba_people_02.jpg
"bin/ycolorgrade" --image tests/toa_heftiba_people.jpg --exposure -0.5 --contrast 0.6 --saturation 0.7 --tint-red 0.995 --tint-green 0.946 --tint-blue 0.829 --grain 0.3 --output out/toa_heftiba_people_03.jpg
"bin/ycolorgrade" --image tests/toa_heftiba_people.jpg --mosaic 16 --grid 16 --output out/toa_heftiba_people_04.jpg