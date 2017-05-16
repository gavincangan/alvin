#!/bin/bash
ffmpeg -r 194 -i default/0/%09d.png -pix_fmt yuv420p $1
