@echo off

cd /d "%~dp0"

"pyinstaller.exe" "main.py" --noconfirm
pause

exit
