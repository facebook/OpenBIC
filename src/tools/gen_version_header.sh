GIT_VERSION="$(git describe --abbrev=8 --dirty --always --tags)"
echo "#define GIT_VERSION \"$GIT_VERSION\"" > include/version.h
