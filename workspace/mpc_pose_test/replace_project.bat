@echo off
setlocal ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

set "SOURCE_FILE=.project_rel"
set "TARGET_FILE=.project"
set "TMP_FILE=%TARGET_FILE%.tmp"
set "TOKEN=PARENT-2-PROJECT_LOC"
set "NAME_TOKEN=REPLACE_WITH_PROJECT_NAME"

REM === Step 1: Extract the first <name> value from .project ===
set "PROJ_NAME="
if exist "%TARGET_FILE%" (
    for /f "tokens=*" %%A in ('findstr "<name>" "%TARGET_FILE%"') do (
        if not defined PROJ_NAME (
            set "LINE=%%A"
            for /f "tokens=2 delims=<>" %%B in ("!LINE!") do (
                set "PROJ_NAME=%%B"
            )
        )
    )
)

REM === Step 2: Compute absolute path two levels up ===
pushd "%CD%\..\.."
set "REPLACEMENT=%CD%"
popd
set "REPLACEMENT=!REPLACEMENT:\=/!"

REM === Step 3: Process the replacement ===
set "TMP2_FILE=%TMP_FILE%.2"
set "NAME_REPLACED=0"

(
    for /f "usebackq delims=" %%L in ("%SOURCE_FILE%") do (
        set "LINE=%%L"
        
        REM Replace the path token
        set "LINE=!LINE:%TOKEN%=%REPLACEMENT%!"
        
        REM Replace the name token if PROJ_NAME was found (only first occurrence)
        if defined PROJ_NAME (
            if "!NAME_REPLACED!"=="0" (
                set "TEMP_LINE=!LINE!"
                set "TEMP_LINE=!TEMP_LINE:%NAME_TOKEN%=__FOUND__!"
                if not "!TEMP_LINE!"=="!LINE!" (
                    set "LINE=!LINE:%NAME_TOKEN%=%PROJ_NAME%!"
                    set "NAME_REPLACED=1"
                )
            )
        )
        
        echo(!LINE!
    )
) > "%TMP2_FILE%"

REM === Step 4: Finalize ===
move /Y "%TMP2_FILE%" "%TARGET_FILE%" >nul
if errorlevel 1 (
    echo Failed to update %TARGET_FILE%.
    exit /b 1
)

echo Success: %TARGET_FILE% updated with Path and Project Name [%PROJ_NAME%].
endlocal