

# Dependencies for FLTK
apt install pkg-config libx11-dev libxft-dev -y

# Dependencies for OpenGL 
apt install libglu1-mesa-dev freeglut3-dev mesa-common-dev -y

# Make and build fltk
cd /opt/
wget https://www.fltk.org/pub/fltk/1.3.5/fltk-1.3.5-source.tar.bz2
tar -xvjf fltk-1.3.5-source.tar.bz2
rm fltk-1.3.5-source.tar.bz2
cd ./fltk*/
./configure --enable-cygwin --enable-shared --enable-threads --enable-xdbe --enable-xft 
make install
# 
# Link local libs for Debian
so=$(lsb_release -i -s)
# 
if test -z "$LD_LIBRARY_PATH" 
then
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
fi
# 
cd ..

wget https://ufpr.dl.sourceforge.net/project/pyfltk/pyfltk/pyFltk-1.3.4.1/pyFltk-1.3.4.1_py3.tar.gz
tar -vzxf pyFltk-1.3.4.1_py3.tar.gz
rm pyFltk-1.3.4.1_py3.tar.gz

# PyFLTK installed
cd pyFltk*/
python3 setup.py build install
python3 setup.py install

cd ~
