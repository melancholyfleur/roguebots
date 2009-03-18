SRCS=sensors.c
TARGET=sensors
ROBOT=hal
CDIR=/opt/gumstix-buildroot/build_arm_nofpu/staging_dir/arm-linux-uclibcgnueabi/bin
CFLAGS=-Os -march=iwmmxt -mtune=iwmmxt -mcpu=iwmmxt -I ../pittCreate-API/include
LIBS=-lm /home/mkk10/pittCreate-API/lib/libpittCreate.a
CPP=${CDIR}/g++
CC=${CDIR}/gcc

${TARGET}:
		${CC} ${CFLAGS} -o ${TARGET} ${SRCS} ${LIBS}

install:
		scp ${TARGET} csbot@${ROBOT}:
