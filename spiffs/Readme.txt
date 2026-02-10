This folder contains wav files required for this project.

To generate correct wave suited for ESP32S3 use below command - 
Note: dependency-> ffmpeg, Link-> Link:https://www.gyan.dev/ffmpeg/builds/ffmpeg-git-full.7z)

> ffmpeg -i input.wav -ar 22050 -ac 1 -sample_fmt s16 -acodec pcm_s16le output.wav

Mp3 to Wav - 

> ffmpeg -i input.mp3 -ar 22050 -ac 1 -acodec pcm_s16le output.wav

Check if wav file properties are as expected - 

> ffprobe file.wav

example:
Input #0, wav, from '.\presenseesp.wav':
  Metadata:
    encoder         : Lavf62.3.100
  Duration: 00:00:03.12, bitrate: 353 kb/s
  Stream #0:0: Audio: pcm_s16le ([1][0][0][0] / 0x0001), 22050 Hz, 1 channels, s16, 352 kb/s
  
  
Note: Customer wav audio online tool (text to speech)
https://ttsmaker.com/
https://ttsfree.com/