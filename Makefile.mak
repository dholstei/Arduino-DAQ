##############################################################################
#
#                     nmake file for DLLs
#
##############################################################################

.SUFFIXES: .dll .obj .c

OBJ = obj
SUFFIX = dll

CC = cl
# CFLAGS = /TC -DWIN -D__LCC__ /wd4047 -D_CRT_SECURE_NO_DEPRECATE -DHAVE_ORACLE /MT
CFLAGS = -DWIN
DLLFLAGS = /DLL $(LIB) /DEF:
LINKER = link.exe

#	Windows
SDK_ROOT = "C:\Program Files (x86)\Windows Kits\10
SDK_VER = 10.0.17134.0
MSVC_ROOT = "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Tools\MSVC\14.15.26726
INCLUDES = -I$(SDK_ROOT)\Include\$(SDK_VER)\um"\
		   -I$(SDK_ROOT)\Include\$(SDK_VER)\shared"\
		   -I$(SDK_ROOT)\Include\$(SDK_VER)\ucrt"\
		   -I$(MSVC_ROOT)\include"
LIB=	/LIBPATH:$(SDK_ROOT)\Lib\$(SDK_VER)\um\x86"\
		/LIBPATH:$(SDK_ROOT)\Lib\$(SDK_VER)\ucrt\x86"\
		/LIBPATH:$(MSVC_ROOT)\lib\x86"
LIBS = "msvcrt.lib" "advapi32.lib" "user32.lib" "Ws2_32.lib"

.c.obj:
  $(CC) $(INCLUDES) -c $(CFLAGS) $*.c

.obj.dll:
  $(LINKER) /out:$@ $(DLLFLAGS)$*_exports.def $*.obj $(LIBS)
		   
all: comm.$(SUFFIX) crc.$(SUFFIX)

clean:
  del *.obj *.dll *.lib *.exp

echo:
  echo "dgh"

test:  comm.$(SUFFIX)
  $(CC) $(INCLUDES) -c $(CFLAGS) $@.c
  $(LINKER) /out:$@.exe $(DLLFLAGS) $@.obj comm.lib
  $@.exe

