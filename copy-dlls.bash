
drive_win_msys=/c/msys64
drive_win_gnulinux=/mnt/ntfs/msys64

# drive_win=$drive_win_gnulinux
drive_win=$drive_win_msys

cp $drive_win/mingw64/bin/SDL2.dll .
cp $drive_win/mingw64/bin/libgcc_s_seh-1.dll .
cp $drive_win/mingw64/bin/libstdc++-6.dll .
cp $drive_win/mingw64/bin/libwinpthread-1.dll .
