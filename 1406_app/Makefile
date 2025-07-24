DIR_INC = ./inc
DIR_INC += -I./src/op_common
# DIR_SRC = ./src
DIR_BIN = .

ver=0.90_B

SUBDIRS := $(shell find . -type d ! -path "./.*")

# 获取所有.c文件
# DIR_SRC := $(foreach dir,$(SUBDIRS),$(wildcard $(dir)/*.c))
DIR_SRC := $(shell find src -type f -name '*.c')



# SOURCE  := $(wildcard ${DIR_SRC}/*.c)
SOURCE := $(DIR_SRC)

OBJS    := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCE)))
  
TARGET  := prj1406-demo-v$(ver)
ALL=$(TARGET) #$(TARGET-DEBUG)

CC      := aarch64-linux-gnu-gcc
LIBS    := -L ./lib/
DEFINES := -DPF_CAN=29 -DAF_CAN=PF_CAN
INCLUDE := -I. -I${DIR_INC}

CFLAGS  := -g -Wall -O2  $(DEFINES) $(INCLUDE) $(LIBS) -static
CFLAGS += -DAPP_VERSION="\"v$(ver)\""
CXXFLAGS:= $(CFLAGS) -DHAVE_CONFIG_H


CFLAGS_SYSLOG=-DSYSLOG

.PHONY : everything objs clean veryclean rebuild
  
everything : $(ALL)
  
all : $(ALL)

#objs : $(OBJS)
send:
	scp $(TARGET) root@10.0.3.222:/home/root

%.o: %.c
	$(CC)  $(CFLAGS) $(CFLAGS_SYSLOG) -c $< -o $@

rebuild: veryclean everything

clean :
	rm -fr *.so
	rm -fr ./*/*.o
	rm -fr *.o
	rm -rf prj1406-*
	rm $(OBJS)

  
$(TARGET) : $(OBJS)
	$(CC) $(CXXFLAGS) -o $@ $(OBJS) -lpthread -lsocketcan
	# mv prj1406-soc-post $(TARGET)