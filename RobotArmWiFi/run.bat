@echo off
rem Jalankan server robot arm di Windows. Venv & dependensi disiapkan otomatis.
cd /d "%~dp0"
if not exist .venv-win\Scripts\python.exe (
  echo Menyiapkan venv...
  python -m venv .venv-win || goto :err
  .venv-win\Scripts\pip install -q -r requirements.txt || goto :err
)
.venv-win\Scripts\python server.py %*
pause
exit /b
:err
echo Gagal menyiapkan venv. Pastikan Python sudah terinstall.
pause
