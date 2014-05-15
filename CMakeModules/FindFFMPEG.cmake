# Find the FFmpeg library
#
# Sets
#   FFMPEG_FOUND.  If false, don't try to use ffmpeg
#   FFMPEG_INCLUDE_DIR
#   FFMPEG_LIBRARIES

set( FFMPEG_FOUND "NO" )


find_path( FFMPEG_INCLUDE_DIR ffmpeg/avcodec.h
  /usr/include
  /usr/local/include
)

if( FFMPEG_INCLUDE_DIR )

find_program( FFMPEG_CONFIG ffmpeg-config
  /usr/bin
  /usr/local/bin
  ${HOME}/bin
)

if( FFMPEG_CONFIG )
  exec_program( ${FFMPEG_CONFIG} ARGS "--libs avformat" OUTPUT_VARIABLE FFMPEG_LIBS )
  set( FFMPEG_FOUND "YES" )
  set( FFMPEG_LIBRARIES "${FFMPEG_LIBS}" )
  
else( FFMPEG_CONFIG )

  find_library( FFMPEG_avcodec_LIBRARY avcodec
    /usr/lib
    /usr/local/lib
    /usr/lib64
    /usr/local/lib64
  )

  find_library( FFMPEG_avformat_LIBRARY avformat
    /usr/lib
    /usr/local/lib
    /usr/lib64
    /usr/local/lib64
  )
  
  find_library( FFMPEG_avutil_LIBRARY avutil
    /usr/lib
    /usr/local/lib
    /usr/lib64
    /usr/local/lib64
  )
  
  if( FFMPEG_avcodec_LIBRARY )
  if( FFMPEG_avformat_LIBRARY )

    set( FFMPEG_FOUND "YES" )
    set( FFMPEG_LIBRARIES ${FFMPEG_avformat_LIBRARY} ${FFMPEG_avcodec_LIBRARY} )
    if( FFMPEG_avutil_LIBRARY )
       set( FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_avutil_LIBRARY} )
    endif( FFMPEG_avutil_LIBRARY )

  endif( FFMPEG_avformat_LIBRARY )
  endif( FFMPEG_avcodec_LIBRARY )

endif( FFMPEG_CONFIG )

endif( FFMPEG_INCLUDE_DIR )
