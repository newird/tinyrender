## download
```bash
git clone https://github.com/Cltsu/tinyrender.git
cd tinyrender
```
## install dependencies
```bash
git submodule update --init --recursive
```
## build
```bash
cmake -S . -B build
cd build
make
```
## run
```bash
./tinyrender [path to .obj file]
```
