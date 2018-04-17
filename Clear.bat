@echo Off
del /s /a *.suo *.ncb *.user *.pdb *.netmodule *.aps *.ilk *.sdf *.htm 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\Build3D\Debug" 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\Release" 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\ipch" 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\.vs" 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\Backup" 2>nul
FOR /R . %%d IN (.) DO rd /s /q "%%d\x64" 2>nul

rem 如果Properties文件夹为空，则删除它
rem FOR /R . %%d in (.) do rd /q "%%d\Properties" 2> nul