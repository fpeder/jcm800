#!/bin/bash
ffmpeg -fflags nobuffer -flags low_delay \
  -f s24be -ar 48000 -ac 1 \
  -i "udp://0.0.0.0:48879?fifo_size=1024&overrun_nonfatal=1" \
  -af aresample=async=1 \
  -c:a pcm_f32le -f pulse -device jcm800 "JCM800"
