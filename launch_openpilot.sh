#!/usr/bin/bash
export ATHENA_HOST='ws://opdiy.cn:5566'
export API_HOST='http://opdiy.cn:7777'
export MAPBOX_TOKEN='pk.eyJ1IjoibXJvbmVjYyIsImEiOiJjbHhqbzlkbTYxNXUwMmtzZjdoMGtrZnVvIn0.SC7GNLtMFUGDgC2bAZcKzg'
export SKIP_FW_QUERY=1
exec ./launch_chffrplus.sh
