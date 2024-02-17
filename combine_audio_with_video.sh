data_dir=data/
for block_type in "$data_dir"*
do
  for trial in "$block_type"/*
  do
    echo "$trial"
    python scripts/combine_audio_and_video.py -a "$trial"/audio.wav -v "$trial"/side_camera.mp4 -o "$trial"/camera.mp4
  done
done