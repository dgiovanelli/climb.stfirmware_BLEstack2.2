@echo off
:: ============================
@echo.
@echo Create combined HEX files
@echo.
:: ============================
:: User's guide for hexmerge.py: http://pythonhosted.org/IntelHex/part3-4.html

pushd ..

:: Path names
@set DEST_PATH=app/FlashOnly_ImgB
@set IMG_A_PATH=app/FlashROM_ImgA
@set IMG_B_PATH=app/FlashOnly_ImgB
@set STACK_PATH=stack/FlashROM
@set BIM_PATH=../../../util/bim/cc1350lp/ccs/FlashOnly

:: Set BIM image
@set BIM_IMG=bim_cc1350lp_bim.hex

:: Set application name
@set APP_NAME=dual_image_concept_cc1350lp

:: Calculate file names
@set APP_IMG=%APP_NAME%_app.hex
@set STACK_IMG=%APP_NAME%_stack.hex
@set APP_FULL_IMG=%APP_IMG:app=unified%

:: Application image start at address 0x1000 (page 0 reserved for BIM vector table)
@set BIM_START=0
@set APP_IMG_A_START=D0
@set APP_IMG_B_START=8000

:: Stack image end (page 30 reserved for NVRAM, page 31 for BIM and CCFG)
@set STACK_END=1DFFF
@set BIM_END=1FFFF

:: 1) DIC image (BIM + Image A + Image B + Stack). For download using flash programmer
"C:\Python27\python" "C:\Python27\Scripts\hexmerge.py" -o "%DEST_PATH%/%APP_FULL_IMG%" -r "%BIM_START%:%BIM_END%" "--overlap=error"  "%IMG_A_PATH%/%APP_IMG%":%APP_IMG_A_START%: "%IMG_B_PATH%/%APP_IMG%":%APP_IMG_B_START%:%STACK_END% "%STACK_PATH%/%STACK_IMG%"::%STACK_END% "%BIM_PATH%/%BIM_IMG%":%HEX_START%:%HEX_END%
echo Created %DEST_PATH%\%APP_FULL_IMG%

pause
