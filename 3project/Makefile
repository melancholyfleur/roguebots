SRCS=maze.c PID.c filter.c error.c
TARGET=maze
ROBOT=Johnny5
CDIR=/opt/gumstix-buildroot/build_arm_nofpu/staging_dir/arm-linux-uclibcgnueabi/bin
CFLAGS=-Os -Wall -march=iwmmxt -mtune=iwmmxt -mcpu=iwmmxt -I /usr/share/pittCreate-API/include
LIBS=-lm /usr/share/pittCreate-API/lib/libpittCreate.a
CPP=${CDIR}/g++
CC=${CDIR}/gcc

${TARGET}:
		${CC} ${CFLAGS} -o ${TARGET} ${SRCS} ${LIBS}

install:
		scp ${TARGET} csbot@${ROBOT}:

