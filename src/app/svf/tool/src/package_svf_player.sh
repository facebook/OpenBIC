pyinstaller -F --hidden-import=serial --hidden-import=readline --hidden-import=glob --hidden-import=progressbar --hidden-import=traceback svf_player.py
cp dist/* ../exe/
