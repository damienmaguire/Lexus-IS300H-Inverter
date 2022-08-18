@echo off

cd /d "%~dp0"

set "RUNMODE=1"
echo 1. run
echo 2. help
set /p "RUNMODE=select run mode (default 1):"

if "%RUNMODE%"=="2" (
	"dist/main/main.exe"
	pause
	exit
)

set /p "APPPATH=enter path:"

set "APPCOLUMNS=3"
set /p "APPCOLUMNS=enter columns:"

set "APPDELIMITER=\s"
set /p "APPDELIMITER=enter delimiter:"

set "APPSEPARATOR=,"
set /p "APPSEPARATOR=enter separator:"

"dist/main/main.exe" %APPPATH% %APPCOLUMNS% %APPDELIMITER% %APPSEPARATOR%

pause

exit
